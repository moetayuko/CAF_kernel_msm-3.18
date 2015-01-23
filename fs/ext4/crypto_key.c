/*
 * linux/fs/ext4/crypto_key.c
 *
 * This contains encryption key functions for ext4
 *
 * Written by Michael Halcrow, Ildar Muslukhov, and Uday Savagaonkar, 2015.
 *
 * This has not yet undergone a rigorous security audit.
 */

#include <keys/encrypted-type.h>
#include <keys/user-type.h>
#include <linux/ecryptfs.h>
#include <linux/random.h>
#include <uapi/linux/keyctl.h>

#include "ext4.h"
#include "ext4_crypto.h"
#include "xattr.h"

#define EXT4_WRAPPING_KEYS_CACHE_TIMEOUT_SEC 60

/**
 * ext4_get_key_construction_buffer_size() - gets the key buffer size
 * @policy:     Policy description string (_NOT_ NULL terminated).
 * @policy_len: Size of @policy.
 *
 * Return: Amount of buffer space necessary to construct @policy.
 */
static int ext4_get_key_construction_buffer_size(const char *policy,
						 size_t policy_len)
{
	char *cursor = (char *)policy;
	char *end = (char *)&policy[policy_len];
	char *next;
	int count = 0;

	BUG_ON(!policy);
	BUG_ON(policy_len <= 0);

	do {
		++count;
		next = memchr(cursor, EXT4_POLICY_DELIMITER, end - cursor);
		if (!next)
			next = end;
		cursor = next + EXT4_POLICY_DELIMITER_SIZE;
	} while (cursor < end);

	return count * EXT4_MAX_KEY_SIZE;
}

/**
 * ext4_copy_key_to_buffer() - copies raw key from keyring object
 * @key_ref:    The keyring object containing the raw key.
 * @buffer:     The receiving buffer.
 * @buffer_pos: Offset in @buffer. Modified by this.
 * @buffer_len: Size of @buffer.
 *
 * Return: Zero on success, non-zero otherwise.
 */
static int ext4_copy_key_to_buffer(key_ref_t key_ref, char *buffer,
				   int *buffer_pos, int buffer_len)
{
	struct key *key = key_ref_to_ptr(key_ref);
	struct ecryptfs_auth_tok *auth_tok;

	BUG_ON(!buffer_pos);

	if (WARN_ON_ONCE(key->datalen !=
			 sizeof(struct ecryptfs_auth_tok))) {
		return -EINVAL;
	}
	if (key->type == &key_type_encrypted) {
		auth_tok = (struct ecryptfs_auth_tok *)
			(&((struct encrypted_key_payload *)
			   key->payload.data)->payload_data);
	} else {
		auth_tok = (struct ecryptfs_auth_tok *)
			(&((struct user_key_payload *)
			   key->payload.data)->data);
	}
	if (WARN_ON_ONCE(!(auth_tok->token.password.flags &
			   ECRYPTFS_SESSION_KEY_ENCRYPTION_KEY_SET))) {
		return -EINVAL;
	}
	BUILD_BUG_ON(EXT4_MAX_KEY_SIZE < EXT4_AES_256_XTS_KEY_SIZE);
	BUILD_BUG_ON(ECRYPTFS_MAX_KEY_BYTES < EXT4_AES_256_XTS_KEY_SIZE);
	if (buffer_len <= EXT4_AES_256_XTS_KEY_SIZE + (*buffer_pos))
		return -EINVAL;
	memcpy(&buffer[*buffer_pos],
	       auth_tok->token.password.session_key_encryption_key,
	       EXT4_AES_256_XTS_KEY_SIZE);
	(*buffer_pos) += EXT4_AES_256_XTS_KEY_SIZE;
	return 0;
}

/* TODO(mhalcrow): Figure out what API we're actually supposed to be using */
extern key_ref_t lookup_user_key(key_serial_t id, unsigned long flags,
				 key_perm_t perm);
#define KEY_LOOKUP_CREATE 1

/**
 * ext4_get_key_from_keyring() - retrieves a key from a keyring
 * @keyref:     The key reference.
 * @keyref_len: Size of @keyref.
 *
 * Return: The retrieved key object from the keyring on success. NULL otherwise.
 */
static key_ref_t ext4_get_key_from_keyring(const char *keyref,
					   size_t keyref_len)
{
	char *cursor;
	size_t keyringname_len = 0, keyname_len = 0;
	char keyname[EXT4_KEY_NAME_NULL_TERMINATED_SIZE];
	key_ref_t keyring = NULL;
	key_ref_t key = NULL;

	cursor = memchr(keyref, EXT4_KEYREF_DELIMITER, keyref_len);
	if (!cursor)
		return NULL;
	keyringname_len = cursor - keyref;
	if (keyref[0] != '@')
		return 0;
	cursor += EXT4_KEYREF_DELIMITER_SIZE;
	keyname_len = keyref_len - keyringname_len - EXT4_KEYREF_DELIMITER_SIZE;
	if (!(keyname_len > 0 &&
	      keyname_len < EXT4_KEY_NAME_NULL_TERMINATED_SIZE)) {
		return NULL;
	}
	memcpy(keyname, cursor, keyname_len);
	keyname[keyname_len] = '\0';

	switch (keyringname_len) {
	case 2:
		if (keyref[1] == 't')
			keyring = lookup_user_key(KEY_SPEC_THREAD_KEYRING, 0,
						  KEY_NEED_SEARCH);
		else if (keyref[1] == 'p')
			keyring = lookup_user_key(KEY_SPEC_PROCESS_KEYRING, 0,
						  KEY_NEED_SEARCH);
		else if (keyref[1] == 's')
			keyring = lookup_user_key(KEY_SPEC_SESSION_KEYRING, 0,
						  KEY_NEED_SEARCH);
		else if (keyref[1] == 'u')
			keyring = lookup_user_key(KEY_SPEC_USER_KEYRING, 0,
						  KEY_NEED_SEARCH);
		else if (keyref[1] == 'g')
			keyring = lookup_user_key(KEY_SPEC_GROUP_KEYRING, 0,
						  KEY_NEED_SEARCH);
		break;
	case 3:
		if (keyref[1] == 'u' && keyref[2] == 's')
			keyring = lookup_user_key(KEY_SPEC_USER_SESSION_KEYRING,
						  0, KEY_NEED_SEARCH);
		break;
	default:
		return 0;
	}

	if (!keyring)
		return 0;

	key = keyring_search(keyring, &key_type_user, keyname);
	key_ref_put(keyring);
	if (IS_ERR(key))
		key = NULL;

	return key;
}

/**
 * ext4_construct_wrapping_key() - generates a wrapping key from a policy
 * @policy:       Non-null terminating policy description string.
 * @policy_len:   Size of @policy.
 * @wrapping_key: Buffer to receive the wrapped key.
 *
 * Return: Zero on success, non-zero otherwise.
 * */
static int ext4_construct_wrapping_key(const char *policy, size_t policy_len,
				       char wrapping_key[EXT4_MAX_KEY_SIZE])
{
	char *cursor = (char *)policy;
	char *end = (char *)&policy[policy_len];
	char *buffer = NULL;
	int buffer_pos = 0, buffer_len = 0;
	char *next = NULL;
	key_ref_t key_ref;
	int res = 0;

	buffer_len = ext4_get_key_construction_buffer_size(policy, policy_len);
	buffer = kmalloc(buffer_len, GFP_NOFS);
	if (!buffer) {
		res = -ENOMEM;
		goto out;
	}

	do {
		next = memchr(cursor, EXT4_POLICY_DELIMITER, end - cursor);
		if (!next)
			next = end;
		if (!ext4_is_key_reference_valid(cursor, next - cursor)) {
			res = -EINVAL;
			goto out;
		}
		key_ref = ext4_get_key_from_keyring(cursor, next - cursor);
		if (!key_ref) {
			res = -EACCES;
			goto out;
		}
		res = ext4_copy_key_to_buffer(key_ref, buffer, &buffer_pos,
					      buffer_len);
		key_ref_put(key_ref);
		if (res)
			goto out;

		cursor = next + EXT4_POLICY_DELIMITER_SIZE;
	} while (cursor < end);

	BUILD_BUG_ON(EXT4_MAX_KEY_SIZE < SHA512_DIGEST_SIZE);
	BUG_ON(buffer_pos > buffer_len);
	res = ext4_hash_sha512(buffer, buffer_pos, wrapping_key);

out:
	if (buffer) {
		memset(buffer, 0, buffer_len);
		kfree(buffer);
	}
	return res;
}

/**
 * ext4_hash_and_encode_policy() - fingerprints a policy
 * @policy:             Policy to fingerprint.
 * @policy_len:         Size of @policy.
 * @policy_fingerprint: Buffer that receives the policy fingerprint.
 */
static int ext4_hash_and_encode_policy(
	char *policy, size_t policy_len,
	char policy_fingerprint[EXT4_WRAPPING_POLICY_FP_NULL_TERMINATED_SIZE])
{
	char digest[SHA256_DIGEST_SIZE];
	int res = 0, i;
	static const char *lookup_table = "0123456789abcdef";

	res = ext4_hash_sha256(policy, policy_len, digest);
	if (res)
		return res;
	for (i = 0;
	     i < (EXT4_WRAPPING_POLICY_FP_NULL_TERMINATED_SIZE - 1) / 2;
	     i++) {
		policy_fingerprint[i << 1] = lookup_table[digest[i] >> 4];
		policy_fingerprint[(i << 1) + 1] = lookup_table[digest[i]
								& 0xf];
	}
	policy_fingerprint[i << 1] = '\0';
	return res;
}

/**
 * ext4_get_wrapping_key() - attempts to produce a wrapping key
 * @inode:              The inode for which we are getting the wrapping key.
 * @policy_fingerprint: Policy fingerprint.
 * @wrapping_key:       A buffer to receive the wrapping key.
 *
 * If an empty string is passed for @policy_fingerprint, then we will calculate
 * fingerprint and replace empty string with that value.
 *
 * Return: Zero on success, non-zero otherwise.
 */
static int ext4_get_wrapping_key(
	const struct inode *inode,
	char policy_fingerprint[EXT4_WRAPPING_POLICY_FP_NULL_TERMINATED_SIZE],
	char wrapping_key[EXT4_MAX_KEY_SIZE])
{
	char *buf = NULL;
	int buf_len;
	int res = 0;

	buf_len = ext4_xattr_get((struct inode *)inode,
				 EXT4_XATTR_INDEX_ENCRYPTION,
				 EXT4_XATTR_NAME_ENCRYPTION_POLICY, NULL, 0);
	if (buf_len <= 0) {
		res = -ENOENT;
		goto out;
	}
	buf = kmalloc(buf_len, GFP_NOFS);
	if (!buf) {
		res = -ENOMEM;
		goto out;
	}
	res = ext4_xattr_get((struct inode *)inode, EXT4_XATTR_INDEX_ENCRYPTION,
			     EXT4_XATTR_NAME_ENCRYPTION_POLICY, buf, buf_len);
	if (res != buf_len) {
		res = -EINVAL;
		goto out;
	}
	if (policy_fingerprint[0] == '\0') {
		res = ext4_hash_and_encode_policy(buf, buf_len,
						  policy_fingerprint);
		if (res)
			goto out;
	}
	res = ext4_construct_wrapping_key(buf, buf_len, wrapping_key);
out:
	kfree(buf);
	return res;
}

/**
 * ext4_unwrap_key() - Unwraps the encryption key for the inode
 * @wrapped_key_packet:      The wrapped encryption key packet.
 * @wrapped_key_packet_size: The wrapped encryption key packet size.
 * @key:                     The encryption key to fill in with unwrapped data.
 *
 * Uses the wrapped key to unwrap the encryption key.
 *
 * Return: Zero on success, non-zero otherwise.
 */
static int ext4_unwrap_key(const struct inode *inode,
			   const char *wrapped_key_packet,
			   size_t wrapped_key_packet_size,
			   struct ext4_encryption_key *key)
{
	struct ext4_wrapped_key_packet_header *packet_header =
		(struct ext4_wrapped_key_packet_header *)wrapped_key_packet;
	struct ext4_wrapped_key_packet *packet = NULL;
	uint32_t packet_size = ntohl(*(uint32_t *)packet_header->size);
	struct ext4_encryption_key_packet key_packet;
	char wrapping_key[EXT4_MAX_KEY_SIZE];
	char enc_key[EXT4_AES_256_CTR_KEY_SIZE];
	char int_key[EXT4_HMAC_KEY_SIZE];
	char hmac[EXT4_HMAC_SIZE];
	char hmac_invalid = 0;
	int i;
	int res = 0;

	if (wrapped_key_packet_size < sizeof(packet_size))
		return -EINVAL;
	BUILD_BUG_ON(sizeof(struct ext4_wrapped_key_packet) !=
		     EXT4_FULL_WRAPPED_KEY_PACKET_V0_SIZE);
	if (packet_size != sizeof(struct ext4_wrapped_key_packet))
		return -EINVAL;
	if (wrapped_key_packet_size != packet_size)
		return -EINVAL;
	if (packet_header->type != EXT4_KEY_PACKET_TYPE_WRAPPED_KEY_V0)
		return -EINVAL;
	packet = (struct ext4_wrapped_key_packet *)wrapped_key_packet;
	if (packet->policy_fingerprint[
		    EXT4_WRAPPING_POLICY_FP_NULL_TERMINATED_SIZE - 1] != '\0') {
		return -EINVAL;
	}
	res = ext4_get_wrapping_key(inode,
				    packet->policy_fingerprint,
				    wrapping_key);
	if (res)
		return res;

	/* Always validate the HMAC as soon as we get the key to do so */
	packet->nonce[EXT4_NONCE_SIZE] = EXT4_WRAPPING_INT_DERIVATION_TWEAK;
	res = ext4_hmac_derive_key(wrapping_key, EXT4_AES_256_XTS_KEY_SIZE,
				   packet->nonce,
				   EXT4_DERIVATION_TWEAK_NONCE_SIZE, int_key,
				   EXT4_HMAC_KEY_SIZE);
	if (res)
		goto out;
	res = ext4_hmac_integrity(int_key, EXT4_HMAC_KEY_SIZE,
				  wrapped_key_packet,
				  (EXT4_FULL_WRAPPED_KEY_PACKET_V0_SIZE -
				   EXT4_HMAC_SIZE), hmac, EXT4_HMAC_SIZE);
	memset(int_key, 0, EXT4_HMAC_KEY_SIZE);
	for (i = 0; i < EXT4_HMAC_SIZE; ++i)
		hmac_invalid |= (packet->hmac[i] ^ hmac[i]);
	if (hmac_invalid) {
		printk_ratelimited(
			KERN_ERR
			"%s: Security warning: Wrapped key HMAC check failed\n",
			__func__);
		res = -EINVAL;
		goto out;
	}

	/* The HMAC validated. Decrypt the key packet. */
	packet->nonce[EXT4_NONCE_SIZE] = EXT4_WRAPPING_ENC_DERIVATION_TWEAK;
	res = ext4_hmac_derive_key(wrapping_key, EXT4_AES_256_XTS_KEY_SIZE,
				   packet->nonce,
				   EXT4_DERIVATION_TWEAK_NONCE_SIZE, enc_key,
				   EXT4_AES_256_CTR_KEY_SIZE);
	if (res)
		goto out;
	res = ext4_crypt_wrapper_virt(enc_key, packet->iv,
				      packet->wrapped_key_packet,
				      (char *)&key_packet,
				      EXT4_V0_SERIALIZED_KEY_SIZE, false);
	memset(enc_key, 0, EXT4_AES_256_CTR_KEY_SIZE);
	if (res)
		goto out;
	key->mode = ext4_validate_encryption_mode(
		ntohl(*((uint32_t *)key_packet.mode)));
	if (key->mode == EXT4_ENCRYPTION_MODE_INVALID) {
		res = -EINVAL;
		goto out;
	}
	memcpy(key->raw, key_packet.raw, EXT4_MAX_KEY_SIZE);
	memset(key_packet.raw, 0, EXT4_MAX_KEY_SIZE);
	key->size = ext4_validate_encryption_key_size(
		key->mode, ntohl(*((uint32_t *)key_packet.size)));
	if (!key->size) {
		res = -EINVAL;
		goto out;
	}
out:
	if (res)
		key->mode = EXT4_ENCRYPTION_MODE_INVALID;
	memset(wrapping_key, 0, EXT4_AES_256_XTS_KEY_SIZE);
	return res;
}

/**
 * ext4_wrap_key() - Wraps the encryption key for the inode
 * @wrapped_crypto_key: The buffer into which this writes the wrapped key.
 * @key_packet_size:    The size of the packet.
 * @key:                The encryption key.
 * @inode:              The inode for the encryption key.
 *
 * Generates a wrapped key packet from an encryption key and a wrapping key for
 * an inode.
 *
 * Return: Zero on success, non-zero otherwise.
 */
static int ext4_wrap_key(char *wrapped_key_packet,
			 size_t *key_packet_size,
			 const struct ext4_encryption_key *key,
			 const struct inode *inode)
{
	struct ext4_wrapped_key_packet *packet =
		(struct ext4_wrapped_key_packet *)wrapped_key_packet;
	struct ext4_encryption_key_packet key_packet;
	char wrapping_key[EXT4_MAX_KEY_SIZE];
	char enc_key[EXT4_AES_256_CTR_KEY_SIZE];
	char int_key[EXT4_HMAC_KEY_SIZE];
	int res = 0;

	BUILD_BUG_ON(sizeof(struct ext4_wrapped_key_packet) !=
		     EXT4_FULL_WRAPPED_KEY_PACKET_V0_SIZE);
	if (!wrapped_key_packet) {
		*key_packet_size = EXT4_FULL_WRAPPED_KEY_PACKET_V0_SIZE;
		return 0;
	}
	/* Force fingerprint construction. */
	packet->policy_fingerprint[0] = '\0';
	res = ext4_get_wrapping_key(inode,
				    packet->policy_fingerprint,
				    wrapping_key);
	if (res) {
		ext4_error(inode->i_sb,
			   "%s: ext4_get_wrapping_key() with "
			   "policy_fingerprint [%s] returned %d\n",
			   __func__, packet->policy_fingerprint, res);
		return res;
	}
	BUG_ON(*key_packet_size != EXT4_FULL_WRAPPED_KEY_PACKET_V0_SIZE);

	/* Size, type, nonce, and IV */
	*((uint32_t *)
	  ((struct ext4_wrapped_key_packet_header *)packet->header)->size) =
		htonl(EXT4_FULL_WRAPPED_KEY_PACKET_V0_SIZE);
	((struct ext4_wrapped_key_packet_header *)packet->header)->type =
		EXT4_KEY_PACKET_TYPE_WRAPPED_KEY_V0;
	get_random_bytes(packet->nonce, EXT4_NONCE_SIZE);
	get_random_bytes(packet->iv, EXT4_WRAPPING_IV_SIZE);

	/* Derive the wrapping encryption key from the wrapping key */
	packet->nonce[EXT4_NONCE_SIZE] = EXT4_WRAPPING_ENC_DERIVATION_TWEAK;
	res = ext4_hmac_derive_key(wrapping_key, EXT4_AES_256_XTS_KEY_SIZE,
				   packet->nonce,
				   EXT4_DERIVATION_TWEAK_NONCE_SIZE,
				   enc_key, EXT4_AES_256_CTR_KEY_SIZE);
	if (res) {
		ext4_error(inode->i_sb,
			   "%s: ext4_hmac_derive_key() returned %d\n",
			   __func__, res);
		goto out;
	}

	/* Wrap the data key with the wrapping encryption key */
	*((uint32_t *)key_packet.mode) = htonl(key->mode);
	memcpy(key_packet.raw, key->raw, EXT4_MAX_KEY_SIZE);
	*((uint32_t *)key_packet.size) = htonl(key->size);
	BUILD_BUG_ON(sizeof(struct ext4_encryption_key_packet) !=
		     EXT4_V0_SERIALIZED_KEY_SIZE);
	res = ext4_crypt_wrapper_virt(enc_key, packet->iv, (char *)&key_packet,
				      (char *)&packet->wrapped_key_packet,
				      EXT4_V0_SERIALIZED_KEY_SIZE, true);
	memset(enc_key, 0, EXT4_AES_256_CTR_KEY_SIZE);
	memset(key_packet.raw, 0, EXT4_MAX_KEY_SIZE);
	if (res) {
		ext4_error(inode->i_sb,
			   "%s: ext4_crypt_wrapper_virt() returned %d\n",
			   __func__, res);
		goto out;
	}

	/* Calculate the HMAC over the entire packet (except, of
	 * course, the HMAC buffer at the end) */
	packet->nonce[EXT4_NONCE_SIZE] = EXT4_WRAPPING_INT_DERIVATION_TWEAK;
	res = ext4_hmac_derive_key(wrapping_key, EXT4_AES_256_XTS_KEY_SIZE,
				   packet->nonce,
				   EXT4_DERIVATION_TWEAK_NONCE_SIZE,
				   int_key, EXT4_HMAC_KEY_SIZE);
	if (res) {
		ext4_error(inode->i_sb,
			   "%s: ext4_hmac_derive_key() returned %d\n",
			   __func__, res);
		goto out;
	}
	BUILD_BUG_ON(EXT4_FULL_WRAPPED_KEY_PACKET_V0_SIZE < EXT4_HMAC_SIZE);
	res = ext4_hmac_integrity(int_key, EXT4_HMAC_KEY_SIZE,
				  wrapped_key_packet,
				  (EXT4_FULL_WRAPPED_KEY_PACKET_V0_SIZE -
				   EXT4_HMAC_SIZE), packet->hmac,
				  EXT4_HMAC_SIZE);
	packet->nonce[EXT4_NONCE_SIZE] = 0;
	/* to catch decryption bugs */
	memset(int_key, 0, EXT4_HMAC_KEY_SIZE);
out:
	memset(wrapping_key, 0, EXT4_AES_256_XTS_KEY_SIZE);
	if (res)
		ext4_error(inode->i_sb, "%s: returning %d\n", __func__, res);
	return res;
}

/**
 * ext4_generate_encryption_key() - generates an encryption key
 * @dentry: The dentry containing the encryption key this will set.
 */
static void ext4_generate_encryption_key(struct inode *inode)
{
	struct ext4_inode_info *ei = EXT4_I(inode);
	struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
	struct ext4_encryption_key *key = &ei->i_encryption_key;

	if (S_ISREG(inode->i_mode))
		key->mode = sbi->s_file_encryption_mode;
	else if (S_ISDIR(inode->i_mode))
		key->mode = sbi->s_dir_encryption_mode;
	else {
		printk(KERN_ERR "ext4 crypto: Unsupported inode type.\n");
		BUG();
	}
	key->size = ext4_encryption_key_size(key->mode);
	BUG_ON(!key->size);
	get_random_bytes(key->raw, key->size);
}

/**
 * ext4_set_crypto_key() - generates and sets the encryption key for the inode
 * @inode: The inode for the encryption key.
 *
 * Return: Zero on success, non-zero otherwise.
 */
static int ext4_set_crypto_key_inode(struct inode *inode)
{
	char root_packet[EXT4_PACKET_SET_V0_MAX_SIZE];
	char *wrapped_key_packet = &root_packet[EXT4_PACKET_HEADER_SIZE];
	struct ext4_wrapped_key_packet *key_packet =
		(struct ext4_wrapped_key_packet *)wrapped_key_packet;
	char wrapping_key[EXT4_MAX_KEY_SIZE];
	size_t wrapped_key_packet_size = EXT4_FULL_WRAPPED_KEY_PACKET_V0_SIZE;
	size_t root_packet_size = (EXT4_PACKET_HEADER_SIZE +
				   wrapped_key_packet_size);
	struct ext4_inode_info *ei = EXT4_I(inode);
	int res = 0;

	/* Before generating a key for an inode, we must make sure that we have
	 * all pieces required by the encryption policy available. To check
	 * this, we attempt to obtain the key. */
	res = ext4_get_wrapping_key(inode, key_packet->policy_fingerprint,
				    wrapping_key);
	if (res)
		goto out;

	ext4_generate_encryption_key(inode);
	res = ext4_wrap_key(wrapped_key_packet, &wrapped_key_packet_size,
			    &ei->i_encryption_key, inode);
	if (res) {
		ext4_error(inode->i_sb,
			   "%s: ext4_wrap_key() returned %d\n", __func__,
			   res);
		goto out;
	}
	root_packet[0] = EXT4_PACKET_SET_VERSION_V0;
	BUILD_BUG_ON(EXT4_PACKET_SET_V0_MAX_SIZE !=
		     (EXT4_PACKET_HEADER_SIZE +
		      EXT4_FULL_WRAPPED_KEY_PACKET_V0_SIZE));
	BUG_ON(sizeof(root_packet) != root_packet_size);

try_again:
	res = ext4_xattr_set(inode,
			     EXT4_XATTR_INDEX_ENCRYPTION,
			     EXT4_XATTR_NAME_ENCRYPTION_CONTEXT,
			     root_packet, root_packet_size, 0);
	if (res) {
		ext4_error(inode->i_sb,
			   "%s: ext4_xattr_set() returned %d\n", __func__,
			   res);
	}
out:
	if (res) {
		if (res == -EINTR)
			goto try_again;
		ei->i_encryption_key.mode = EXT4_ENCRYPTION_MODE_INVALID;
	}
	return res;
}

/**
 * ext4_set_crypto_key() - generates and sets the encryption key for the dentry
 * @dentry: The dentry for the encryption key.
 *
 * Return: Zero on success, non-zero otherwise.
 */
int ext4_set_crypto_key(struct dentry *dentry)
{
	return ext4_set_crypto_key_inode(dentry->d_inode);
}

/**
 * ext4_get_root_packet() - reads the root packet
 * @inode:            The inode containing the root packet.
 * @root_packet:      The root packet.
 * @root_packet_size: The size of the root packet. Set by this if
 *                    root_packet == NULL.
 *
 * Return: Zero on success, non-zero otherwise.
 */
static int ext4_get_root_packet(struct inode *inode, char *root_packet,
				size_t *root_packet_size)
{
	int res = ext4_xattr_get(inode,
				 EXT4_XATTR_INDEX_ENCRYPTION,
				 EXT4_XATTR_NAME_ENCRYPTION_CONTEXT,
				 NULL, 0);
	if (res < 0)
		return res;
	if (!root_packet) {
		*root_packet_size = res;
		return 0;
	}
	if (res != *root_packet_size)
		return -ENODATA;
	res = ext4_xattr_get(inode,
			     EXT4_XATTR_INDEX_ENCRYPTION,
			     EXT4_XATTR_NAME_ENCRYPTION_CONTEXT,
			     root_packet, res);
	if (root_packet[0] != EXT4_PACKET_SET_VERSION_V0) {
		printk_ratelimited(
			KERN_ERR
			"%s: Expected root packet version %d; got %d\n",
			__func__, EXT4_PACKET_SET_VERSION_V0, root_packet[0]);
		return -EINVAL;
	}
	return 0;
}

/**
 * ext4_is_encryption_key_set() - checks for encryption key xattr on inode
 * @inode: The inode to check.
 *
 * Return: Zero if xattr is not set, non-zero otherwise.
 * */
int ext4_is_encryption_key_set(struct inode *inode)
{
	return ext4_xattr_get(inode, EXT4_XATTR_INDEX_ENCRYPTION,
			      EXT4_XATTR_NAME_ENCRYPTION_CONTEXT, NULL, 0) > 0;
}

/**
 * ext4_get_crypto_key_inode() - gets the encryption key for the inode
 * @inode: Inode of a directory or a file for which the encryption key sought.
 *
 * Return: Zero on success, non-zero otherwise.
 */
int ext4_get_crypto_key_inode(struct inode *inode)
{
	char root_packet[EXT4_PACKET_SET_V0_MAX_SIZE];
	char *wrapped_key_packet = &root_packet[EXT4_PACKET_HEADER_SIZE];
	size_t wrapped_key_packet_size = EXT4_FULL_WRAPPED_KEY_PACKET_V0_SIZE;
	size_t root_packet_size = (EXT4_PACKET_HEADER_SIZE +
				   wrapped_key_packet_size);
	struct ext4_inode_info *ei = EXT4_I(inode);
	int res;

	/* Check for special case when a policy is set, but the key is not.
	 * NOTE: This case happens when someone (e.g., root) sets a policy on
	 * an empty directory, but user later provides the key. */
	if (!ext4_is_encryption_key_set(inode)) {
		res = ext4_set_crypto_key_inode(inode);
		if (res)
			goto out;
	}

	res = ext4_get_root_packet(inode, root_packet, &root_packet_size);
	if (res) {
		/* We must have a root packet for regular files. */
		if (S_ISREG(inode->i_mode))
			goto out;
		/* For directories we can proceed without a root packet, since
		 * we will show hashed filenames in that case. */
		ei->i_encryption_key.mode = EXT4_ENCRYPTION_MODE_PLAINTEXT;
		res = 0;
		goto out;
	}
	res = ext4_unwrap_key(inode, wrapped_key_packet,
			      EXT4_FULL_WRAPPED_KEY_PACKET_V0_SIZE,
			      &ei->i_encryption_key);
	if (res && S_ISDIR(inode->i_mode)) {
		/* Key unwrap is fine for directories; we'll fall back to
		 * showing hashed filenames. */
		ei->i_encryption_key.size = 0;
		res = 0;
	}
out:
	if (res)
		ei->i_encryption_key.mode = EXT4_ENCRYPTION_MODE_INVALID;
	return res;
}

/**
 * ext4_get_crypto_key() - gets the encryption key for the inode
 * @file: The file for the encryption key.
 *
 * Return: Zero on success, non-zero otherwise.
 */
int ext4_get_crypto_key(const struct file *file)
{
	return ext4_get_crypto_key_inode(file->f_mapping->host);
}
