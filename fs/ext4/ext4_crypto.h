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

#endif	/* _EXT4_CRYPTO_H */
