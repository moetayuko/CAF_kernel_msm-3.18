/*
 * linux/fs/ext4/crypto_policy.c
 *
 * This contains encryption policy functions for ext4
 *
 * Written by Ildar Muslukhov <ildar@csnow.ca>, 2014.
 *
 * This has not yet undergone a rigorous security audit.
 *
 * A policy is specified in a keyringid1.keyname1:...:keyringidN.keynameN, where
 * keyringidI is a 1 or 2 char id of a keyring (e.g., "us" for User Session
 * Keyring) and keyname is a key description for keyring (must be printable).
 * A key reference is a tuple of (keyringid, keyname).
 */

#include <linux/string.h>
#include <linux/types.h>

#include "ext4.h"
#include "ext4_crypto.h"
#include "xattr.h"

/**
 * ext4_is_keydesc_str_valid() - Validates the key descriptor string
 * @str:  The key name string, not NULL-terminated.
 * @size: The size of @str.
 *
 * Return: Zero if invalid, non-zero otherwise.
 */
static int ext4_is_keydesc_str_valid(const char *str, size_t size)
{
	size_t i;

	if (!(size >= EXT4_KEY_DESCRIPTOR_MIN_SIZE &&
	      size <= EXT4_KEY_DESCRIPTOR_MAX_SIZE)) {
		return 0;
	}

	for (i = 0; i < size; ++i) {
		if (!((str[i] >= '0' && str[i] <= '9') ||
		      (str[i] >= 'A' && str[i] <= 'Z') ||
		      (str[i] >= 'a' && str[i] <= 'z'))) {
			return 0;
		}
	}

	return 1;
}

/**
 * ext4_is_key_reference_valid() - Validates a key reference string
 * @keyref:     The key reference string. Format: "@KEYRING.KEYDESCRIPTOR".
 * @keyref_len: The length of @keyref, not including any NULL terminator.
 *
 * Return: Zero if invalid, non-zero otherwise.
 */
int ext4_is_key_reference_valid(const char *keyref, size_t keyref_len)
{
	char *delimiter;
	const char *keydescriptor;
	size_t keylocation_name_len;
	size_t keydescriptor_name_len;

	BUG_ON(keyref_len <= 0);
	BUG_ON(!keyref);

	delimiter = memchr(keyref, EXT4_KEYREF_DELIMITER, keyref_len);
	if (!delimiter)
		return 0;

	keylocation_name_len = delimiter - keyref;
	keydescriptor = &keyref[keylocation_name_len +
				EXT4_KEYREF_DELIMITER_SIZE];
	keydescriptor_name_len = &keyref[keyref_len] - keydescriptor;

	if ((strncmp(keyref, "@u", keylocation_name_len) == 0 ||
	     strncmp(keyref, "@us", keylocation_name_len) == 0 ||
	     strncmp(keyref, "@s", keylocation_name_len) == 0 ||
	     strncmp(keyref, "@g", keylocation_name_len) == 0 ||
	     strncmp(keyref, "@p", keylocation_name_len) == 0 ||
	     strncmp(keyref, "@t", keylocation_name_len) == 0) &&
	    ext4_is_keydesc_str_valid(keydescriptor, keydescriptor_name_len)) {
		return 1;
	}

	return 0;
}

/**
 * ext4_is_encryption_policy_valid() - Validates encryption policy
 * @policy:     A policy string, not NULL-terminated.
 * @policy_len: Size of the policy string.
 *
 * Return: Zero if invalid, non-zero otherwise.
 */
int ext4_is_encryption_policy_valid(const char *policy, size_t policy_len)
{
	char *cursor = (char *)policy;
	char *end = (char *)&policy[policy_len];
	char *next;

	BUG_ON(!policy);
	BUG_ON(policy_len <= 0);

	do {
		next = memchr(cursor, EXT4_POLICY_DELIMITER, end - cursor);
		if (!next)
			next = end;
		if (!ext4_is_key_reference_valid(cursor, next - cursor))
			return 0;
		cursor = next + EXT4_POLICY_DELIMITER_SIZE;
	} while (cursor < end);

	return 1;
}

/**
 * ext4_is_encryption_policy_set() - Checks if inode has encryption policy
 * @inode: The inode to check.
 *
 * Return: Zero if not set, non-zero otherwise.
 */
int ext4_is_encryption_policy_set(struct inode *inode)
{
	return ext4_xattr_get(inode, EXT4_XATTR_INDEX_ENCRYPTION,
			      EXT4_XATTR_NAME_ENCRYPTION_POLICY, NULL, 0) > 0;
}

/**
 * ext4_inherit_policy() - Sets a child policy from its parent
 * @parent: Parent inode from which the policy is inherited.
 * @child:  Child inode that inherits the policy from @parent.
 *
 * Return: Zero on success, non-zero otherwise
 */
int ext4_inherit_policy(struct inode *parent, struct inode *child)
{
	int res = 0;
	char *buf = NULL;
	int buf_len = ext4_xattr_get(parent, EXT4_XATTR_INDEX_ENCRYPTION,
				     EXT4_XATTR_NAME_ENCRYPTION_POLICY, NULL,
				     0);

	if (buf_len <= 0)
		return -ENOENT;
	buf = kmalloc(buf_len, GFP_NOFS);
	if (!buf)
		return -ENOMEM;
	res = ext4_xattr_get(parent, EXT4_XATTR_INDEX_ENCRYPTION,
			     EXT4_XATTR_NAME_ENCRYPTION_POLICY, buf, buf_len);
	if (res != buf_len) {
		res = -EINVAL;
		goto out;
	}
	res = ext4_xattr_set(child, EXT4_XATTR_INDEX_ENCRYPTION,
			     EXT4_XATTR_NAME_ENCRYPTION_POLICY, buf, buf_len,
			     0);

out:
	kfree(buf);
	return res;
}
