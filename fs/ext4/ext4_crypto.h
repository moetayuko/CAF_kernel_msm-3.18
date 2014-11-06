/*
 * linux/fs/ext4/ext4_crypto.h
 *
 * This contains encryption header content for ext4
 *
 * Written by Michael Halcrow, 2014.
 */

#ifndef _EXT4_CRYPTO_H
#define _EXT4_CRYPTO_H

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
#define EXT4_MAX_IV_SIZE AES_BLOCK_SIZE
#define EXT4_MAX_AUTH_SIZE EXT4_AES_256_GCM_AUTH_SIZE

/* The metadata directory is only necessary only for the sibling file
 * directory under the mount root, which will be replaced by per-block
 * metadata when it's ready. */
#define EXT4_METADATA_DIRECTORY_NAME ".ext4_crypt_data"
#define EXT4_METADATA_DIRECTORY_NAME_SIZE 16

/**
 * Packet format:
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

/**
 * Wrapped key packet format:
 *  4 bytes: Size of packet (inclusive of these 4 bytes)
 *  1 byte: Packet type/version (0x00)
 *   17 bytes: NULL-terminated wrapping key signature (printable)
 *   13 bytes: Derivation nonce (last byte ignored)
 *   16 bytes: IV
 *   Variable bytes: Serialized key, AES-256-CTR encrypted
 *   12 bytes: HMAC-SHA1(everything preceding)
 */
#define EXT4_KEY_PACKET_TYPE_WRAPPED_KEY_V0 ((char)0x00)
#define EXT4_WRAPPING_KEY_SIG_NULL_TERMINATED_SIZE (ECRYPTFS_SIG_SIZE_HEX + 1)
#define EXT4_WRAPPING_IV_SIZE 16

/* These #defines may seem redundant to the sizeof the structs below
 * them. Since naively changing the structs can result in nasty bugs
 * that might have security implications, we use the explict sizes
 * together with BUILD_BUG_ON() to help avoid mistakes. */
#define EXT4_V0_SERIALIZED_KEY_SIZE (sizeof(uint32_t) + \
				     EXT4_MAX_KEY_SIZE + \
				     sizeof(uint32_t))
#define EXT4_WRAPPED_KEY_PACKET_V0_SIZE ( \
		EXT4_WRAPPING_KEY_SIG_NULL_TERMINATED_SIZE + \
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

/* Don't change this without also changing the packet type. Serialized
 * packets are cast directly into this struct. */
struct ext4_encryption_key_packet {
	char mode[sizeof(uint32_t)]; /* Network byte order */
	char raw[EXT4_MAX_KEY_SIZE];
	char size[sizeof(uint32_t)]; /* Network byte order */
} __attribute__((__packed__));

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
};

struct ext4_encryption_key {
	uint32_t mode;
	char raw[EXT4_MAX_KEY_SIZE];
	uint32_t size;
};

/* Don't change this without also changing the packet type. Serialized
 * packets are cast directly into this struct. */
struct ext4_wrapped_key_packet {
	char size[sizeof(uint32_t)]; /* Network byte order */
	char type;
	char sig[EXT4_WRAPPING_KEY_SIG_NULL_TERMINATED_SIZE];
	char nonce[EXT4_DERIVATION_TWEAK_NONCE_SIZE];
	char iv[EXT4_WRAPPING_IV_SIZE];
	char wrapped_key_packet[sizeof(struct ext4_encryption_key_packet)];
	char hmac[EXT4_HMAC_SIZE];
} __attribute__((__packed__));

struct ext4_encryption_wrapper_desc {
	char wrapping_key_sig[ECRYPTFS_SIG_SIZE_HEX + 1];
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
	default:
		BUG();
	}
	return 0;
}

#endif	/* _EXT4_CRYPTO_H */
