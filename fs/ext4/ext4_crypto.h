/*
 * linux/fs/ext4/ext4_crypto.h
 *
 * This contains encryption header content for ext4
 *
 * Written by Michael Halcrow, 2015.
 *
 * Filename encryption additions
 *	Uday Savagaonkar, 2015
 * Encryption policy handling additions
 *	Ildar Muslukhov, 2014
 */

#ifndef _EXT4_CRYPTO_H
#define _EXT4_CRYPTO_H

/* xattr names for protector packet (context) and policy description */
#define EXT4_XATTR_NAME_ENCRYPTION_POLICY "policy"
#define EXT4_XATTR_NAME_ENCRYPTION_CONTEXT "context"

#define EXT4_KEYREF_DELIMITER ((char)'.')
#define EXT4_KEYREF_DELIMITER_SIZE 1
#define EXT4_POLICY_DELIMITER ((char)':')
#define EXT4_POLICY_DELIMITER_SIZE 1

/* A key descriptor consists of two parts (a) a references of a location where
 * to fetch the key from, and (b) a key fingerprint or an alias up to 16
 * characters. */
#define EXT4_KEY_DESCRIPTOR_MAX_SIZE 16
#define EXT4_KEY_DESCRIPTOR_MIN_SIZE 4

int ext4_is_encryption_policy_valid(const char *policy, size_t policy_len);
int ext4_is_encryption_policy_set(struct inode *inode);
int ext4_is_key_reference_valid(const char *keyref, size_t keyref_len);
int ext4_inherit_policy(struct inode *parent, struct inode *child);

#include <crypto/sha.h>

/* Encryption parameters */
#define EXT4_AES_256_XTS_KEY_SIZE 64
#define EXT4_XTS_TWEAK_SIZE 16
#define EXT4_AES_256_CTR_KEY_SIZE 32
#define EXT4_AES_256_ECB_KEY_SIZE 32
#define EXT4_HMAC_KEY_SIZE 12
#define EXT4_HMAC_SIZE 12
#define EXT4_NONCE_SIZE 12
#define EXT4_DERIVATION_TWEAK_SIZE 1
#define EXT4_DERIVATION_TWEAK_NONCE_SIZE (EXT4_NONCE_SIZE + \
					  EXT4_DERIVATION_TWEAK_SIZE)
#define EXT4_WRAPPING_ENC_DERIVATION_TWEAK 'e'
#define EXT4_WRAPPING_INT_DERIVATION_TWEAK 'i'
#define EXT4_AES_256_XTS_RANDOMIV_HMAC_SHA1_KEY_SIZE \
	(EXT4_AES_256_XTS_KEY_SIZE + EXT4_HMAC_KEY_SIZE)
#define EXT4_AES_256_GCM_KEY_SIZE 32
#define EXT4_AES_256_GCM_AUTH_SIZE 16
#define EXT4_GCM_ASSOC_DATA_SIZE sizeof(pgoff_t)
#define EXT4_PAGE_REGION_INDEX_SHIFT 16 /* 2**16-sized regions */
#define EXT4_MAX_KEY_SIZE EXT4_AES_256_XTS_RANDOMIV_HMAC_SHA1_KEY_SIZE
#define EXT4_AES_256_CBC_KEY_SIZE 32
#define EXT4_MAX_IV_SIZE AES_BLOCK_SIZE
#define EXT4_MAX_AUTH_SIZE EXT4_AES_256_GCM_AUTH_SIZE

/**
 * Packet header format:
 *  4 bytes: Size of packet (inclusive of these 4 bytes)
 *  1 byte: Packet type/version
 *   Variable bytes: Packet content (may contain nested packets)
 *
 * Packets may be nested. The top-level packet is the "packet set".
 */
#define EXT4_PACKET_SET_VERSION_V0 ((char)0x00)
#define EXT4_PACKET_SET_VERSION_SIZE 1
#define EXT4_PACKET_SIZE_SIZE 4
#define EXT4_PACKET_TYPE_SIZE 1
#define EXT4_PACKET_HEADER_SIZE (EXT4_PACKET_SIZE_SIZE + EXT4_PACKET_TYPE_SIZE)

/* Packet header struct. All packets are dereferenced to that structure first,
 * and only then to specific type. Change carefully. */
struct ext4_wrapped_key_packet_header {
	char size[sizeof(uint32_t)]; /* Network byte order */
	char type;
} __attribute__((__packed__));

/**
 * Wrapped key packet format:
 *  4 bytes: Size of packet (inclusive of these 4 bytes)
 *  1 byte: Packet type/version (0x00)
 *   17 bytes: NULL-terminated wrapping policy signature (printable)
 *   13 bytes: Derivation nonce (last byte ignored)
 *   16 bytes: IV
 *   Variable bytes: Serialized key, AES-256-CTR encrypted
 *   12 bytes: HMAC-SHA1(everything preceding)
 */
#define EXT4_KEY_PACKET_TYPE_WRAPPED_KEY_V0 ((char)0x00)
#define EXT4_KEY_NAME_SIZE 16
#define EXT4_KEY_NAME_NULL_TERMINATED_SIZE (EXT4_KEY_NAME_SIZE + 1)
#define EXT4_WRAPPING_POLICY_FINGERPRINT_SIZE 16
#define EXT4_WRAPPING_POLICY_FP_NULL_TERMINATED_SIZE \
	(EXT4_WRAPPING_POLICY_FINGERPRINT_SIZE + 1)
#define EXT4_WRAPPING_IV_SIZE 16

/* These #defines may seem redundant to the sizeof the structs below
 * them. Since naively changing the structs can result in nasty bugs
 * that might have security implications, we use the explicit sizes
 * together with BUILD_BUG_ON() to help avoid mistakes. */
#define EXT4_V0_SERIALIZED_KEY_SIZE (sizeof(uint32_t) + \
				     EXT4_MAX_KEY_SIZE + \
				     sizeof(uint32_t))
#define EXT4_WRAPPED_KEY_PACKET_V0_SIZE ( \
		EXT4_WRAPPING_POLICY_FP_NULL_TERMINATED_SIZE + \
		EXT4_DERIVATION_TWEAK_NONCE_SIZE + \
		EXT4_WRAPPING_IV_SIZE + \
		EXT4_V0_SERIALIZED_KEY_SIZE + \
		EXT4_HMAC_SIZE)

#define EXT4_FULL_WRAPPED_KEY_PACKET_V0_SIZE ((uint32_t)( \
		EXT4_PACKET_HEADER_SIZE +		  \
		EXT4_WRAPPED_KEY_PACKET_V0_SIZE))

/* V0 supports only one key in a fixed xattr space. If/when compelling
 * requirements come along, future versions may be able to use
 * (non-xattr) metadata storage to store an arbitrary number of
 * wrapped keys. In the meantime, we won't spend the code complexity
 * budget on supporting multiple wrapped keys. */
#define EXT4_PACKET_SET_V0_MAX_WRAPPED_KEYS 1
#define EXT4_PACKET_SET_V0_MAX_SIZE ((uint32_t)(	\
		EXT4_PACKET_HEADER_SIZE +		\
		(EXT4_FULL_WRAPPED_KEY_PACKET_V0_SIZE * \
		EXT4_PACKET_SET_V0_MAX_WRAPPED_KEYS)))

/**
 * If you change the existing modes (order or type), you'll need to
 * change the packet type too.
 */
enum ext4_encryption_mode {
	EXT4_ENCRYPTION_MODE_INVALID = 0,
	EXT4_ENCRYPTION_MODE_AES_256_XTS,
	EXT4_ENCRYPTION_MODE_AES_256_GCM,
	EXT4_ENCRYPTION_MODE_HMAC_SHA1,
	EXT4_ENCRYPTION_MODE_AES_256_XTS_RANDOM_IV_HMAC_SHA1,
	EXT4_ENCRYPTION_MODE_AES_256_CBC,
	EXT4_ENCRYPTION_MODE_PLAINTEXT
};

/* Packed encryption key */
struct ext4_encryption_key_packet {
	char mode[sizeof(uint32_t)]; /* Network byte order */
	char raw[EXT4_MAX_KEY_SIZE];
	char size[sizeof(uint32_t)]; /* Network byte order */
} __attribute__((__packed__));

struct ext4_encryption_key {
	uint32_t mode;
	char raw[EXT4_MAX_KEY_SIZE];
	uint32_t size;
};

/* Don't change this without also changing the packet type. Serialized
 * packets are cast directly into this struct. */
struct ext4_wrapped_key_packet {
	char header[sizeof(struct ext4_wrapped_key_packet_header)];
	char policy_fingerprint[EXT4_WRAPPING_POLICY_FP_NULL_TERMINATED_SIZE];
	char nonce[EXT4_DERIVATION_TWEAK_NONCE_SIZE];
	char iv[EXT4_WRAPPING_IV_SIZE];
	char wrapped_key_packet[sizeof(struct ext4_encryption_key_packet)];
	char hmac[EXT4_HMAC_SIZE];
} __attribute__((__packed__));

struct ext4_encryption_wrapper_desc {
	char wrapping_key_sig[EXT4_KEY_NAME_NULL_TERMINATED_SIZE];
};

#define EXT4_CTX_REQUIRES_FREE_ENCRYPT_FL		0x00000001
#define EXT4_BOUNCE_PAGE_REQUIRES_FREE_ENCRYPT_FL	0x00000002

struct ext4_crypto_ctx {
	struct crypto_tfm *tfm;         /* Crypto API context */
	struct page *bounce_page;	/* Ciphertext page on write path */
	struct page *control_page;	/* Original page on write path */
	struct bio *bio;		/* The bio for this context */
	struct work_struct work;	/* Work queue for read complete path */
	struct list_head free_list;	/* Free list */
	int flags;			/* Flags */
	enum ext4_encryption_mode mode; /* Encryption mode for tfm */
	atomic_t dbg_refcnt;            /* TODO(mhalcrow): Remove for release */
};

static inline int ext4_encryption_key_size(enum ext4_encryption_mode mode)
{
	switch (mode) {
	case EXT4_ENCRYPTION_MODE_AES_256_XTS:
		return EXT4_AES_256_XTS_KEY_SIZE;
	case EXT4_ENCRYPTION_MODE_AES_256_GCM:
		return EXT4_AES_256_GCM_KEY_SIZE;
	case EXT4_ENCRYPTION_MODE_HMAC_SHA1:
		return EXT4_HMAC_KEY_SIZE;
	case EXT4_ENCRYPTION_MODE_AES_256_XTS_RANDOM_IV_HMAC_SHA1:
		return EXT4_AES_256_XTS_RANDOMIV_HMAC_SHA1_KEY_SIZE;
	case EXT4_ENCRYPTION_MODE_AES_256_CBC:
		return EXT4_AES_256_CBC_KEY_SIZE;
	default:
		BUG();
	}
	return 0;
}

struct ext4_hmac_result {
	struct completion completion;
	int res;
};

int ext4_hash_sha256(const char *src, size_t src_size,
		     char digest[SHA256_DIGEST_SIZE]);
int ext4_hash_sha512(const char *src, size_t src_size,
		     char digest[SHA512_DIGEST_SIZE]);
int ext4_hmac_derive_key(const char *key, size_t key_size, const char *src,
			 size_t src_size, char *dst, size_t dst_size);
int ext4_hmac_integrity(const char *key, size_t key_size, const char *src,
			size_t src_size, char *dst, size_t dst_size);
int ext4_crypt_wrapper_virt(const char *enc_key, const char *iv,
			    const char *src_virt, char *dst_virt, size_t size,
			    bool enc);
uint32_t ext4_validate_encryption_mode(uint32_t mode);
uint32_t ext4_validate_encryption_key_size(uint32_t mode, uint32_t size);
int ext4_is_encryption_key_set(struct inode *inode);
extern struct workqueue_struct *mpage_read_workqueue;
int ext4_allocate_crypto(size_t num_crypto_pages, size_t num_crypto_ctxs);
void ext4_delete_crypto(void);
struct ext4_crypto_ctx *ext4_get_crypto_ctx(
	bool with_page, const struct ext4_encryption_key *key);
void ext4_release_crypto_ctx(struct ext4_crypto_ctx *ctx);
void set_bh_to_page(struct buffer_head *head, struct page *page);
struct page *ext4_encrypt(struct ext4_crypto_ctx *ctx,
			  struct page *plaintext_page);
int ext4_decrypt(struct ext4_crypto_ctx *ctx, struct page *page);
int ext4_get_crypto_key(const struct file *file);
int ext4_set_crypto_key(struct dentry *dentry);

#endif	/* _EXT4_CRYPTO_H */
