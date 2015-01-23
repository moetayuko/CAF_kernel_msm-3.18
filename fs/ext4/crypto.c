/*
 * linux/fs/ext4/crypto.c
 *
 * This contains encryption functions for ext4
 *
 * Written by Michael Halcrow, 2014.
 *
 * Filename encryption additions
 *	Uday Savagaonkar, 2014
 * Encryption policy handling additions
 *	Ildar Muslukhov, 2014
 *
 * This has not yet undergone a rigorous security audit.
 *
 * The usage of AES-XTS should conform to recommendations in NIST
 * Special Publication 800-38E. The usage of AES-GCM should conform to
 * the recommendations in NIST Special Publication 800-38D. Further
 * guidance for block-oriented storage is in IEEE P1619/D16. The key
 * derivation code implements an HKDF (see RFC 5869).
 */

#include <crypto/hash.h>
#include <crypto/sha.h>
#include <keys/user-type.h>
#include <keys/encrypted-type.h>
#include <linux/crypto.h>
#include <linux/ecryptfs.h>
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/key.h>
#include <linux/list.h>
#include <linux/mempool.h>
#include <linux/random.h>
#include <linux/scatterlist.h>
#include <linux/spinlock_types.h>

#include "ext4.h"
#include "ext4_crypto.h"
#include "xattr.h"

/* Encryption added and removed here! (L: */

static mempool_t *ext4_bounce_page_pool;

static LIST_HEAD(ext4_free_crypto_ctxs);
static DEFINE_SPINLOCK(ext4_crypto_ctx_lock);

/**
 * ext4_release_crypto_ctx() - Releases an encryption context
 * @ctx: The encryption context to release.
 *
 * If the encryption context was allocated from the pre-allocated pool, returns
 * it to that pool. Else, frees it.
 *
 * If there's a bounce page in the context, this frees that.
 */
void ext4_release_crypto_ctx(struct ext4_crypto_ctx *ctx)
{
	unsigned long flags;

	atomic_dec(&ctx->dbg_refcnt);
	if (ctx->bounce_page) {
		if (ctx->flags & EXT4_BOUNCE_PAGE_REQUIRES_FREE_ENCRYPT_FL)
			__free_page(ctx->bounce_page);
		else
			mempool_free(ctx->bounce_page, ext4_bounce_page_pool);
		ctx->bounce_page = NULL;
	}
	ctx->control_page = NULL;
	if (ctx->flags & EXT4_CTX_REQUIRES_FREE_ENCRYPT_FL) {
		if (ctx->tfm)
			crypto_free_tfm(ctx->tfm);
		kfree(ctx);
	} else {
		spin_lock_irqsave(&ext4_crypto_ctx_lock, flags);
		list_add(&ctx->free_list, &ext4_free_crypto_ctxs);
		spin_unlock_irqrestore(&ext4_crypto_ctx_lock, flags);
	}
}

/**
 * ext4_alloc_and_init_crypto_ctx() - Allocates and inits an encryption context
 * @mask: The allocation mask.
 *
 * Return: An allocated and initialized encryption context on success. An error
 * value or NULL otherwise.
 */
static struct ext4_crypto_ctx *ext4_alloc_and_init_crypto_ctx(gfp_t mask)
{
	struct ext4_crypto_ctx *ctx = kzalloc(sizeof(struct ext4_crypto_ctx),
					      mask);

	if (!ctx)
		return ERR_PTR(-ENOMEM);
	return ctx;
}

/**
 * ext4_get_crypto_ctx() - Gets an encryption context
 * @with_page: If true, allocates and attaches a bounce page.
 * @key:       The encryption key for the context.
 *
 * Allocates and initializes an encryption context.
 *
 * Return: An allocated and initialized encryption context on success; error
 * value or NULL otherwise.
 */
struct ext4_crypto_ctx *ext4_get_crypto_ctx(
	bool with_page, const struct ext4_encryption_key *key)
{
	struct ext4_crypto_ctx *ctx = NULL;
	int res = 0;
	unsigned long flags;

	/* We first try getting the ctx from a free list because in the common
	 * case the ctx will have an allocated and initialized crypto tfm, so
	 * it's probably a worthwhile optimization. For the bounce page, we
	 * first try getting it from the kernel allocator because that's just
	 * about as fast as getting it from a list and because a cache of free
	 * pages should generally be a "last resort" option for a filesystem to
	 * be able to do its job. */
	spin_lock_irqsave(&ext4_crypto_ctx_lock, flags);
	ctx = list_first_entry_or_null(&ext4_free_crypto_ctxs,
				       struct ext4_crypto_ctx, free_list);
	if (ctx)
		list_del(&ctx->free_list);
	spin_unlock_irqrestore(&ext4_crypto_ctx_lock, flags);
	if (!ctx) {
		ctx = ext4_alloc_and_init_crypto_ctx(GFP_NOFS);
		if (IS_ERR(ctx)) {
			res = PTR_ERR(ctx);
			goto out;
		}
		ctx->flags |= EXT4_CTX_REQUIRES_FREE_ENCRYPT_FL;
	} else {
		ctx->flags &= ~EXT4_CTX_REQUIRES_FREE_ENCRYPT_FL;
	}
	atomic_set(&ctx->dbg_refcnt, 0);

	/* Allocate a new Crypto API context if we don't already have one or if
	 * it isn't the right mode. */
	BUG_ON(key->mode == EXT4_ENCRYPTION_MODE_INVALID);
	if (ctx->tfm && (ctx->mode != key->mode)) {
		crypto_free_tfm(ctx->tfm);
		ctx->tfm = NULL;
		ctx->mode = EXT4_ENCRYPTION_MODE_INVALID;
	}
	if (!ctx->tfm) {
		switch (key->mode) {
		case EXT4_ENCRYPTION_MODE_AES_256_XTS:
			ctx->tfm = crypto_ablkcipher_tfm(
				crypto_alloc_ablkcipher("xts(aes)", 0, 0));
			break;
		case EXT4_ENCRYPTION_MODE_AES_256_GCM:
			/* TODO(mhalcrow): AEAD w/ gcm(aes);
			 * crypto_aead_setauthsize() */
		case EXT4_ENCRYPTION_MODE_HMAC_SHA1:
			/* TODO(mhalcrow): AHASH w/ hmac(sha1) */
		case EXT4_ENCRYPTION_MODE_AES_256_XTS_RANDOM_IV_HMAC_SHA1:
			ctx->tfm = ERR_PTR(-ENOTSUPP);
			break;
		default:
			BUG();
		}
		if (IS_ERR_OR_NULL(ctx->tfm)) {
			res = PTR_ERR(ctx->tfm);
			ctx->tfm = NULL;
			goto out;
		}
		ctx->mode = key->mode;
	}
	BUG_ON(key->size != ext4_encryption_key_size(key->mode));

	/* There shouldn't be a bounce page attached to the crypto
	 * context at this point. */
	BUG_ON(ctx->bounce_page);
	if (!with_page)
		goto out;

	/* The encryption operation will require a bounce page. */
	ctx->bounce_page = alloc_page(GFP_NOFS);
	if (!ctx->bounce_page) {
		/* This is a potential bottleneck, but at least we'll have
		 * forward progress. */
		ctx->bounce_page = mempool_alloc(ext4_bounce_page_pool,
						 GFP_NOFS);
		if (WARN_ON_ONCE(!ctx->bounce_page)) {
			ctx->bounce_page = mempool_alloc(ext4_bounce_page_pool,
							 GFP_NOFS | __GFP_WAIT);
		}
		ctx->flags &= ~EXT4_BOUNCE_PAGE_REQUIRES_FREE_ENCRYPT_FL;
	} else {
		ctx->flags |= EXT4_BOUNCE_PAGE_REQUIRES_FREE_ENCRYPT_FL;
	}
out:
	if (res) {
		if (!IS_ERR_OR_NULL(ctx))
			ext4_release_crypto_ctx(ctx);
		ctx = ERR_PTR(res);
	}
	return ctx;
}

struct workqueue_struct *mpage_read_workqueue;

/**
 * ext4_delete_crypto_ctxs() - Deletes/frees all encryption contexts
 */
static void ext4_delete_crypto_ctxs(void)
{
	struct ext4_crypto_ctx *pos, *n;

	list_for_each_entry_safe(pos, n, &ext4_free_crypto_ctxs, free_list) {
		if (pos->bounce_page) {
			if (pos->flags &
			    EXT4_BOUNCE_PAGE_REQUIRES_FREE_ENCRYPT_FL) {
				__free_page(pos->bounce_page);
			} else {
				mempool_free(pos->bounce_page,
					     ext4_bounce_page_pool);
			}
		}
		if (pos->tfm)
			crypto_free_tfm(pos->tfm);
		kfree(pos);
	}
}

/**
 * ext4_allocate_crypto_ctxs() -  Allocates a pool of encryption contexts
 * @num_to_allocate: The number of encryption contexts to allocate.
 *
 * Return: Zero on success, non-zero otherwise.
 */
static int __init ext4_allocate_crypto_ctxs(size_t num_to_allocate)
{
	struct ext4_crypto_ctx *ctx = NULL;

	while (num_to_allocate > 0) {
		ctx = ext4_alloc_and_init_crypto_ctx(GFP_KERNEL);
		if (IS_ERR(ctx))
			break;
		list_add(&ctx->free_list, &ext4_free_crypto_ctxs);
		num_to_allocate--;
	}
	if (IS_ERR(ctx))
		ext4_delete_crypto_ctxs();
	return PTR_ERR_OR_ZERO(ctx);
}

/**
 * ext4_delete_crypto() - Frees all allocated encryption objects
 */
void ext4_delete_crypto(void)
{
	ext4_delete_crypto_ctxs();
	mempool_destroy(ext4_bounce_page_pool);
	destroy_workqueue(mpage_read_workqueue);
}

/**
 * ext4_allocate_crypto() - Allocates encryption objects for later use
 * @num_crypto_pages: The number of bounce pages to allocate for encryption.
 * @num_crypto_ctxs:  The number of encryption contexts to allocate.
 *
 * Return: Zero on success, non-zero otherwise.
 */
int __init ext4_allocate_crypto(size_t num_crypto_pages, size_t num_crypto_ctxs)
{
	int res = 0;

	mpage_read_workqueue = alloc_workqueue("ext4_crypto", WQ_HIGHPRI, 0);
	if (!mpage_read_workqueue) {
		res = -ENOMEM;
		goto fail;
	}
	res = ext4_allocate_crypto_ctxs(num_crypto_ctxs);
	if (res)
		goto fail;
	ext4_bounce_page_pool = mempool_create_page_pool(num_crypto_pages, 0);
	if (!ext4_bounce_page_pool)
		goto fail;
	return 0;
fail:
	ext4_delete_crypto();
	return res;
}

/**
 * ext4_xts_tweak_for_page() - Generates an XTS tweak for a page
 * @xts_tweak: Buffer into which this writes the XTS tweak.
 * @page:      The page for which this generates a tweak.
 *
 * Generates an XTS tweak value for the given page.
 */
static void ext4_xts_tweak_for_page(u8 xts_tweak[EXT4_XTS_TWEAK_SIZE],
				    const struct page *page)
{
	/* Only do this for XTS tweak values. For other modes (CBC,
	 * GCM, etc.), you most like will need to do something
	 * different. */
	BUILD_BUG_ON(EXT4_XTS_TWEAK_SIZE < sizeof(page->index));
	memcpy(xts_tweak, &page->index, sizeof(page->index));
	memset(&xts_tweak[sizeof(page->index)], 0,
	       EXT4_XTS_TWEAK_SIZE - sizeof(page->index));
}

/**
 * set_bh_to_page() - Re-assigns the pages for a set of buffer heads
 * @head: The head of the buffer list to reassign.
 * @page: The page to which to re-assign the buffer heads.
 */
void set_bh_to_page(struct buffer_head *head, struct page *page)
{
	struct buffer_head *bh = head;

	do {
		set_bh_page(bh, page, bh_offset(bh));
		if (PageDirty(page))
			set_buffer_dirty(bh);
		if (!bh->b_this_page)
			bh->b_this_page = head;
	} while ((bh = bh->b_this_page) != head);
}

struct ext4_crypt_result {
	struct completion completion;
	int res;
};

/**
 * ext4_crypt_complete() - The completion callback for page encryption
 * @req: The asynchronous encryption request context
 * @res: The result of the encryption operation
 */
static void ext4_crypt_complete(struct crypto_async_request *req, int res)
{
	struct ext4_crypt_result *ecr = req->data;

	if (res == -EINPROGRESS)
		return;
	ecr->res = res;
	complete(&ecr->completion);
}

/**
 * ext4_prep_pages_for_write() - Prepares pages for write
 * @ciphertext_page: Ciphertext page that will actually be written.
 * @plaintext_page:  Plaintext page that acts as a control page.
 * @ctx:             Encryption context for the pages.
 */
static void ext4_prep_pages_for_write(struct page *ciphertext_page,
				      struct page *plaintext_page,
				      struct ext4_crypto_ctx *ctx)
{
	SetPageDirty(ciphertext_page);
	SetPagePrivate(ciphertext_page);
	ctx->control_page = plaintext_page;
	set_page_private(ciphertext_page, (unsigned long)ctx);
	set_bh_to_page(page_buffers(plaintext_page), ciphertext_page);
}

/**
 * ext4_xts_encrypt() - Encrypts a page using AES-256-XTS
 * @ctx:            The encryption context.
 * @plaintext_page: The page to encrypt. Must be locked.
 *
 * Allocates a ciphertext page and encrypts plaintext_page into it using the ctx
 * encryption context. Uses AES-256-XTS.
 *
 * Called on the page write path.
 *
 * Return: An allocated page with the encrypted content on success. Else, an
 * error value or NULL.
 */
static struct page *ext4_xts_encrypt(struct ext4_crypto_ctx *ctx,
				     struct page *plaintext_page)
{
	struct page *ciphertext_page = ctx->bounce_page;
	u8 xts_tweak[EXT4_XTS_TWEAK_SIZE];
	struct ablkcipher_request *req = NULL;
	struct ext4_crypt_result ecr;
	struct scatterlist dst, src;
	struct ext4_inode_info *ei = EXT4_I(plaintext_page->mapping->host);
	struct crypto_ablkcipher *atfm = __crypto_ablkcipher_cast(ctx->tfm);
	int res = 0;

	BUG_ON(!ciphertext_page);
	BUG_ON(!ctx->tfm);
	BUG_ON(ei->i_encryption_key.mode != EXT4_ENCRYPTION_MODE_AES_256_XTS);
	crypto_ablkcipher_clear_flags(atfm, ~0);
	crypto_tfm_set_flags(ctx->tfm, CRYPTO_TFM_REQ_WEAK_KEY);

	/* Since in AES-256-XTS mode we only perform one cryptographic operation
	 * on each block and there are no constraints about how many blocks a
	 * single key can encrypt, we directly use the inode master key */
	res = crypto_ablkcipher_setkey(atfm, ei->i_encryption_key.raw,
				       ei->i_encryption_key.size);
	req = ablkcipher_request_alloc(atfm, GFP_NOFS);
	if (!req) {
		printk_ratelimited(KERN_ERR
				   "%s: crypto_request_alloc() failed\n",
				   __func__);
		ciphertext_page = ERR_PTR(-ENOMEM);
		goto out;
	}
	ablkcipher_request_set_callback(
		req, CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
		ext4_crypt_complete, &ecr);
	ext4_xts_tweak_for_page(xts_tweak, plaintext_page);
	sg_init_table(&dst, 1);
	sg_set_page(&dst, ciphertext_page, PAGE_CACHE_SIZE, 0);
	sg_init_table(&src, 1);
	sg_set_page(&src, plaintext_page, PAGE_CACHE_SIZE, 0);
	ablkcipher_request_set_crypt(req, &src, &dst, PAGE_CACHE_SIZE,
				     xts_tweak);
	res = crypto_ablkcipher_encrypt(req);
	if (res == -EINPROGRESS || res == -EBUSY) {
		BUG_ON(req->base.data != &ecr);
		wait_for_completion(&ecr.completion);
		res = ecr.res;
	}
	ablkcipher_request_free(req);
	if (res) {
		printk_ratelimited(
			KERN_ERR
			"%s: crypto_ablkcipher_encrypt() returned %d\n",
			__func__, res);
		ciphertext_page = ERR_PTR(res);
		goto out;
	}
out:
	return ciphertext_page;
}

/**
 * ext4_encrypt() - Encrypts a page
 * @ctx:            The encryption context.
 * @plaintext_page: The page to encrypt. Must be locked.
 *
 * Allocates a ciphertext page and encrypts plaintext_page into it using the ctx
 * encryption context.
 *
 * Called on the page write path.
 *
 * Return: An allocated page with the encrypted content on success. Else, an
 * error value or NULL.
 */
struct page *ext4_encrypt(struct ext4_crypto_ctx *ctx,
			  struct page *plaintext_page)
{
	struct page *ciphertext_page = NULL;

	BUG_ON(!PageLocked(plaintext_page));
	switch (ctx->mode) {
	case EXT4_ENCRYPTION_MODE_AES_256_XTS:
		ciphertext_page = ext4_xts_encrypt(ctx, plaintext_page);
		break;
	case EXT4_ENCRYPTION_MODE_AES_256_GCM:
		/* TODO(mhalcrow): We'll need buffers for the
		 * generated IV and/or auth tag for this mode and the
		 * ones below */
	case EXT4_ENCRYPTION_MODE_HMAC_SHA1:
	case EXT4_ENCRYPTION_MODE_AES_256_XTS_RANDOM_IV_HMAC_SHA1:
		ciphertext_page = ERR_PTR(-ENOTSUPP);
		break;
	default:
		BUG();
	}
	if (!IS_ERR_OR_NULL(ciphertext_page))
		ext4_prep_pages_for_write(ciphertext_page, plaintext_page, ctx);
	return ciphertext_page;
}

/**
 * ext4_xts_decrypt() - Decrypts a page using AES-256-XTS
 * @ctx:  The encryption context.
 * @page: The page to decrypt. Must be locked.
 *
 * Return: Zero on success, non-zero otherwise.
 */
static int ext4_xts_decrypt(struct ext4_crypto_ctx *ctx, struct page *page)
{
	u8 xts_tweak[EXT4_XTS_TWEAK_SIZE];
	struct ablkcipher_request *req = NULL;
	struct ext4_crypt_result ecr;
	struct scatterlist sg;
	struct ext4_inode_info *ei = EXT4_I(page->mapping->host);
	struct crypto_ablkcipher *atfm = __crypto_ablkcipher_cast(ctx->tfm);
	int res = 0;

	BUG_ON(!ctx->tfm);
	BUG_ON(ei->i_encryption_key.mode != EXT4_ENCRYPTION_MODE_AES_256_XTS);
	crypto_ablkcipher_clear_flags(atfm, ~0);
	crypto_tfm_set_flags(ctx->tfm, CRYPTO_TFM_REQ_WEAK_KEY);

	/* Since in AES-256-XTS mode we only perform one cryptographic operation
	 * on each block and there are no constraints about how many blocks a
	 * single key can encrypt, we directly use the inode master key */
	res = crypto_ablkcipher_setkey(atfm, ei->i_encryption_key.raw,
				       ei->i_encryption_key.size);
	req = ablkcipher_request_alloc(atfm, GFP_NOFS);
	if (!req) {
		res = -ENOMEM;
		goto out;
	}
	ablkcipher_request_set_callback(
		req, CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
		ext4_crypt_complete, &ecr);
	ext4_xts_tweak_for_page(xts_tweak, page);
	sg_init_table(&sg, 1);
	sg_set_page(&sg, page, PAGE_CACHE_SIZE, 0);
	ablkcipher_request_set_crypt(req, &sg, &sg, PAGE_CACHE_SIZE, xts_tweak);
	res = crypto_ablkcipher_decrypt(req);
	if (res == -EINPROGRESS || res == -EBUSY) {
		BUG_ON(req->base.data != &ecr);
		wait_for_completion(&ecr.completion);
		res = ecr.res;
	}
	ablkcipher_request_free(req);
out:
	if (res)
		printk_ratelimited(KERN_ERR "%s: res = %d\n", __func__, res);
	return res;
}

/**
 * ext4_decrypt() - Decrypts a page in-place
 * @ctx:  The encryption context.
 * @page: The page to decrypt. Must be locked.
 *
 * Decrypts page in-place using the ctx encryption context.
 *
 * Called from the read completion callback.
 *
 * Return: Zero on success, non-zero otherwise.
 */
int ext4_decrypt(struct ext4_crypto_ctx *ctx, struct page *page)
{
	int res = 0;

	BUG_ON(!PageLocked(page));
	switch (ctx->mode) {
	case EXT4_ENCRYPTION_MODE_AES_256_XTS:
		res = ext4_xts_decrypt(ctx, page);
		break;
	case EXT4_ENCRYPTION_MODE_AES_256_GCM:
	case EXT4_ENCRYPTION_MODE_HMAC_SHA1:
	case EXT4_ENCRYPTION_MODE_AES_256_XTS_RANDOM_IV_HMAC_SHA1:
		res = -ENOTSUPP;
		break;
	default:
		BUG();
	}
	return res;
}

/**
 * ext4_validate_encryption_mode() - Validates the encryption key mode
 * @mode: The key mode to validate.
 *
 * Return: The validated key mode. EXT4_ENCRYPTION_MODE_INVALID if invalid.
 */
uint32_t ext4_validate_encryption_mode(uint32_t mode)
{
	switch (mode) {
	case EXT4_ENCRYPTION_MODE_AES_256_XTS:
		return mode;
	case EXT4_ENCRYPTION_MODE_AES_256_CBC:
		return mode;
	default:
		break;
	}
	return EXT4_ENCRYPTION_MODE_INVALID;
}

/**
 * ext4_validate_encryption_key_size() - Validate the encryption key size
 * @mode: The key mode.
 * @size: The key size to validate.
 *
 * Return: The validated key size for @mode. Zero if invalid.
 */
uint32_t ext4_validate_encryption_key_size(uint32_t mode, uint32_t size)
{
	if (size == ext4_encryption_key_size(mode))
		return size;
	return 0;
}

/**
 * ext4_hmac_complete() - Completion for async HMAC
 * @req: The async request.
 * @res: The result of the HMAC operation.
 */
void ext4_hmac_complete(struct crypto_async_request *req, int res)
{
	struct ext4_hmac_result *ehr = req->data;

	if (res == -EINPROGRESS)
		return;
	ehr->res = res;
	complete(&ehr->completion);
}

/**
 * ext4_hmac() - Generates an HMAC
 * @derivation: If true, derive a key. Else, generate an integrity HMAC.
 * @key:        The HMAC key.
 * @key_size:   The size of @key.
 * @src:        The data to HMAC.
 * @src_size:   The size of @src.
 * @dst:        The target buffer for the generated HMAC.
 * @dst_size:   The size of @dst.
 *
 * Return: Zero on success, non-zero otherwise.
 */
static int ext4_hmac(bool derivation, const char *key, size_t key_size,
		     const char *src, size_t src_size, char *dst,
		     size_t dst_size)
{
	struct scatterlist sg;
	struct ahash_request *req = NULL;
	struct ext4_hmac_result ehr;
	char hmac[SHA512_DIGEST_SIZE];
	struct crypto_ahash *tfm = crypto_alloc_ahash(derivation ?
						      "hmac(sha512)" :
						      "hmac(sha1)", 0, 0);
	int res = 0;

	BUG_ON(dst_size > SHA512_DIGEST_SIZE);
	if (IS_ERR(tfm)) {
		printk(KERN_ERR "%s: crypto_alloc_ahash() returned %ld\n",
		       __func__, PTR_ERR(tfm));
		return PTR_ERR(tfm);
	}
	req = ahash_request_alloc(tfm, GFP_NOFS);
	if (!req) {
		res = -ENOMEM;
		goto out;
	}
	ahash_request_set_callback(req,
				   (CRYPTO_TFM_REQ_MAY_BACKLOG |
				    CRYPTO_TFM_REQ_MAY_SLEEP),
				   ext4_hmac_complete, &ehr);

	res = crypto_ahash_setkey(tfm, key, key_size);
	if (res) {
		printk(KERN_ERR "%s: crypto_ahash_setkey() returned %d\n",
		       __func__, res);
		goto out;
	}
	sg_init_one(&sg, src, src_size);
	ahash_request_set_crypt(req, &sg, hmac, src_size);
	init_completion(&ehr.completion);
	res = crypto_ahash_digest(req);
	if (res == -EINPROGRESS || res == -EBUSY) {
		BUG_ON(req->base.data != &ehr);
		wait_for_completion(&ehr.completion);
		res = ehr.res;
	}
	if (res) {
		printk(KERN_ERR "%s: crypto_ahash_digest() returned %d\n",
		       __func__, res);
		goto out;
	}
	memcpy(dst, hmac, dst_size);
out:
	crypto_free_ahash(tfm);
	if (req)
		ahash_request_free(req);
	if (res)
		printk(KERN_ERR "%s: returning %d\n", __func__, res);
	return res;
}

/**
 * ext4_hmac_derive_key() - Generates an HMAC for an key derivation (HKDF)
 * @key:      The master key.
 * @key_size: The size of @key.
 * @src:      The derivation data.
 * @src_size: The size of @src.
 * @dst:      The target buffer for the derived key.
 * @dst_size: The size of @dst.
 *
 * Return: Zero on success, non-zero otherwise.
 */
int ext4_hmac_derive_key(const char *key, size_t key_size, const char *src,
			 size_t src_size, char *dst, size_t dst_size)
{
	return ext4_hmac(true, key, key_size, src, src_size, dst, dst_size);
}

/**
 * ext4_hmac_integrity() - Generates an HMAC for an integrity measurement
 * @key:      The HMAC key.
 * @key_size: The size of @key.
 * @src:      The data to generate the HMAC over.
 * @src_size: The size of @src.
 * @dst:      The target buffer for the HMAC.
 * @dst_size: The size of @dst.
 *
 * Return: Zero on success, non-zero otherwise.
 */
int ext4_hmac_integrity(const char *key, size_t key_size, const char *src,
			size_t src_size, char *dst, size_t dst_size)
{
	return ext4_hmac(false, key, key_size, src, src_size, dst, dst_size);
}

/**
 * ext4_crypt_wrapper_virt() - Encrypts a key
 * @enc_key:  The wrapping key.
 * @iv:       The initialization vector for the key encryption.
 * @src_virt: The source key object to wrap.
 * @dst_virt: The buffer for the wrapped key object.
 * @size:     The size of the key object (identical for wrapped or unwrapped).
 * @enc:      If 0, decrypt. Else, encrypt.
 *
 * Uses the wrapped key to unwrap the encryption key.
 *
 * Return: Zero on success, non-zero otherwise.
 */
int ext4_crypt_wrapper_virt(const char *enc_key, const char *iv,
			    const char *src_virt, char *dst_virt, size_t size,
			    bool enc)
{
	struct scatterlist dst, src;
	struct blkcipher_desc desc = {
		.flags = CRYPTO_TFM_REQ_MAY_SLEEP
	};
	int res = 0;

	desc.tfm = crypto_alloc_blkcipher("ctr(aes)", 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(desc.tfm)) {
		printk(KERN_ERR "%s: crypto_alloc_blkcipher() returned %ld\n",
		       __func__, PTR_ERR(desc.tfm));
		return PTR_ERR(desc.tfm);
	}
	if (!desc.tfm)
		return -ENOMEM;
	crypto_blkcipher_set_flags(desc.tfm, CRYPTO_TFM_REQ_WEAK_KEY);
	sg_init_one(&dst, dst_virt, size);
	sg_init_one(&src, src_virt, size);
	crypto_blkcipher_set_iv(desc.tfm, iv, EXT4_WRAPPING_IV_SIZE);
	res = crypto_blkcipher_setkey(desc.tfm, enc_key,
				      EXT4_AES_256_CTR_KEY_SIZE);
	if (res) {
		printk(KERN_ERR "%s: crypto_blkcipher_setkey() returned %d\n",
		       __func__, res);
		goto out;
	}
	if (enc)
		res = crypto_blkcipher_encrypt(&desc, &dst, &src, size);
	else
		res = crypto_blkcipher_decrypt(&desc, &dst, &src, size);
	if (res) {
		printk(KERN_ERR "%s: crypto_blkcipher_*crypt() returned %d\n",
		       __func__, res);
	}
out:
	crypto_free_blkcipher(desc.tfm);
	return res;
}

/**
 * ext4_hash_sha256() - Generates a SHA-256 hash
 * @src:      The input buffer to hash.
 * @src_size: Length of the input buffer.
 * @digest:   The digest buffer into which to write the hash.
 *
 * Return: 0 on success, non-zero otherwise.
 */
int ext4_hash_sha256(const char *src, size_t src_size,
		     char digest[SHA256_DIGEST_SIZE])
{
	struct scatterlist sg;
	struct crypto_hash *tfm = crypto_alloc_hash("sha256", 0, 0);
	struct hash_desc desc = {
		.tfm = tfm,
		.flags = CRYPTO_TFM_REQ_MAY_SLEEP
	};
	int res = 0;

	BUG_ON(src_size > PAGE_CACHE_SIZE);
	sg_init_one(&sg, src, src_size);
	res = crypto_hash_init(&desc);
	if (res)
		goto out;
	res = crypto_hash_update(&desc, &sg, src_size);
	if (res)
		goto out;
	res = crypto_hash_final(&desc, digest);
	if (res)
		goto out;
out:
	crypto_free_hash(tfm);
	return res;
}

/**
 * ext4_hash_sha512() - Generates a SHA-512 hash
 * @src:      The input buffer to hash.
 * @src_size: Length of the input buffer.
 * @digest:   The digest buffer into which to write the hash.
 *
 * Return: 0 on success, non-zero otherwise.
 */
int ext4_hash_sha512(const char *src, size_t src_size,
		     char digest[SHA512_DIGEST_SIZE])
{
	struct scatterlist sg;
	struct crypto_hash *tfm = crypto_alloc_hash("sha512", 0, 0);
	struct hash_desc desc = {
		.tfm = tfm,
		.flags = CRYPTO_TFM_REQ_MAY_SLEEP
	};
	int res = 0;

	BUG_ON(src_size > PAGE_CACHE_SIZE);
	sg_init_one(&sg, src, src_size);
	res = crypto_hash_init(&desc);
	if (res)
		goto out;
	res = crypto_hash_update(&desc, &sg, src_size);
	if (res)
		goto out;
	res = crypto_hash_final(&desc, digest);
	if (res)
		goto out;
out:
	crypto_free_hash(tfm);
	return res;
}
