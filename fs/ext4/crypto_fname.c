/*
 * linux/fs/ext4/ext4_fname_crypto.c
 *
 * This contains functions for filename crypto management in ext4
 *
 * Written by Uday Savagaonkar, 2014.
 *
 * This has not yet undergone a rigorous security audit.
 *
 */

#include <crypto/hash.h>
#include <crypto/sha.h>
#include <keys/encrypted-type.h>
#include <keys/user-type.h>
#include <linux/crypto.h>
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/key.h>
#include <linux/key.h>
#include <linux/list.h>
#include <linux/mempool.h>
#include <linux/random.h>
#include <linux/scatterlist.h>
#include <linux/spinlock_types.h>

#include "ext4.h"
#include "ext4_crypto.h"
#include "xattr.h"

/**
 * copy_bufs_to_page() - copies cstr bufs into a page
 * @pagebuf: Receives the concatenated contents of the @bufs array.
 * @bufs:    An array of cstr buffers.
 * @tlen:    Number of elements in the @bufs array.
 */
static void copy_bufs_to_page(char *pagebuf, const struct ext4_cstr *bufs,
			      u32 tlen)
{
	u32 tcopied = 0, remaining = tlen, bytes_this_iter;
	int i = 0;

	while (remaining) {
		bytes_this_iter = (remaining > bufs[i].len ?
				   bufs[i].len : remaining);
		memcpy(pagebuf + tcopied, bufs[i].name, bytes_this_iter);
		tcopied += bytes_this_iter;
		remaining -= bytes_this_iter;
		++i;
	}
}

/**
 * map_bufs_to_sg() - maps buffers into a scatterlist
 *
 * Maps a list of buffers to a scatterlist of pages.
 *
 * Return: The number of entries used in the scatterlist. Caller can
 * use this return value to further extend the scatterlist. If the input list
 * does not have sufficient space, -ENOMEM is returned.
 */
static int map_bufs_to_sg(struct scatterlist *lst, const u32 lstlen,
			  const struct ext4_fname_crypto_buf_desc *buf_arr,
			  const u32 num_buf)
{
	phys_addr_t pa;
	u32 offset, len, entries_used;
	struct page *pg;
	int i;
	char *buf;
	u32 size;

	sg_init_table(lst, lstlen);
	entries_used = 0;
	for (i = 0; i < num_buf; ++i) {
		buf = buf_arr[i].buf;
		size = buf_arr[i].size;
		while (size && entries_used < lstlen) {
			pg = virt_to_page(buf);
			pa = (phys_addr_t)buf;
			offset = pa & (PAGE_SIZE - 1);
			len = (PAGE_SIZE-offset < size ?
			       PAGE_SIZE-offset : size);
			sg_set_page(&lst[entries_used], pg, len, offset);

			buf += len;
			size -= len;
			++entries_used;
		}
		if (size)
			return -ENOMEM;
	}

	return entries_used;
}

struct ext4_dir_crypt_result {
	struct completion completion;
	int res;
};

/**
 * ext4_dir_crypt_complete() -
 */
static void ext4_dir_crypt_complete(struct crypto_async_request *req, int res)
{
	struct ext4_dir_crypt_result *ecr = req->data;

	if (res == -EINPROGRESS)
		return;
	ecr->res = res;
	complete(&ecr->completion);
}

/**
 * ext4_cbc_encrypt() -
 */
static int ext4_cbc_encrypt(struct ext4_fname_crypto_ctx *ctx,
			    struct ext4_cstr *oname,
			    const struct ext4_cstr *ibufs, u32 ilen)
{
	struct ablkcipher_request *req = NULL;
	struct ext4_dir_crypt_result ecr;
	struct scatterlist sg[1];
	struct crypto_ablkcipher *tfm = ctx->ctfm;
	int res = 0;
	int ciphertext_len;
	char iv[EXT4_CRYPTO_BLOCK_SIZE];
	char *workbuf;

	/* Allocate request */
	req = ablkcipher_request_alloc(tfm, GFP_NOFS);
	if (!req) {
		printk_ratelimited(
		    KERN_ERR "%s: crypto_request_alloc() failed\n", __func__);
		return -ENOMEM;
	}
	ablkcipher_request_set_callback(req,
		CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
		ext4_dir_crypt_complete, &ecr);

	ciphertext_len = ext4_fname_crypto_round_up(ilen,
			EXT4_CRYPTO_BLOCK_SIZE);
	ciphertext_len = (ciphertext_len > ctx->lim)
			? ctx->lim : ciphertext_len;

	/* Map the workpage */
	workbuf = kmap(ctx->workpage);

	/* Copy the input */
	copy_bufs_to_page(workbuf, ibufs, ilen);
	if (ilen < ciphertext_len)
		memset(workbuf+ilen, 0, ciphertext_len-ilen);

	/* Initialize IV */
	memset(iv, 0, EXT4_CRYPTO_BLOCK_SIZE);

	/* Create encryption request */
	sg_init_table(sg, 1);
	sg_set_page(sg, ctx->workpage, PAGE_SIZE, 0);
	ablkcipher_request_set_crypt(req, sg, sg, ciphertext_len, iv);
	res = crypto_ablkcipher_encrypt(req);
	if (res == -EINPROGRESS || res == -EBUSY) {
		BUG_ON(req->base.data != &ecr);
		wait_for_completion(&ecr.completion);
		res = ecr.res;
		if (res)
			goto out;
	}
	/* Copy the result to output */
	memcpy(oname->name, workbuf, ciphertext_len);
	oname->len = ciphertext_len;
	res = ciphertext_len;

out:
	kunmap(ctx->workpage);
	ablkcipher_request_free(req);
	if (res < 0) {
		printk_ratelimited(
		    KERN_ERR "%s: Error (error code %d)\n", __func__, res);
	}
	return res;
}

/**
 * ext4_cbc_decrypt() -
 */
static int ext4_cbc_decrypt(struct ext4_fname_crypto_ctx *ctx,
			    struct ext4_cstr *oname,
			    const struct ext4_cstr *ibufs, u32 ilen)
{
	struct ablkcipher_request *req = NULL;
	struct ext4_dir_crypt_result ecr;
	struct scatterlist sg[1];
	struct crypto_ablkcipher *tfm = ctx->ctfm;
	int res = 0;
	char iv[EXT4_CRYPTO_BLOCK_SIZE];
	char *workbuf;

	if ((ilen&(EXT4_CRYPTO_BLOCK_SIZE-1)) != 0) {
		printk_ratelimited(
		    KERN_ERR "%s: ilen is not integer multiple of EXT4_CRYPTO_BLOCK_SIZE\n",
		    __func__);
		return -EIO;
	}
	/* Allocate request */
	req = ablkcipher_request_alloc(tfm, GFP_NOFS);
	if (!req) {
		printk_ratelimited(
		    KERN_ERR "%s: crypto_request_alloc() failed\n",  __func__);
		return -ENOMEM;
	}
	ablkcipher_request_set_callback(req,
		CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
		ext4_dir_crypt_complete, &ecr);

	/* Map the workpage */
	workbuf = kmap(ctx->workpage);

	/* Copy the input */
	copy_bufs_to_page(workbuf, ibufs, ilen);

	/* Initialize IV */
	memset(iv, 0, EXT4_CRYPTO_BLOCK_SIZE);

	/* Create encryption request */
	sg_init_table(sg, 1);
	sg_set_page(sg, ctx->workpage, PAGE_SIZE, 0);
	ablkcipher_request_set_crypt(req, sg, sg, ilen, iv);
	res = crypto_ablkcipher_decrypt(req);
	if (res == -EINPROGRESS || res == -EBUSY) {
		BUG_ON(req->base.data != &ecr);
		wait_for_completion(&ecr.completion);
		res = ecr.res;
		if (res)
			goto out;
	}
	/* Copy the result to output */
	memcpy(oname->name, workbuf, ilen);
	oname->len = ilen;
	res = ilen;

out:
	kunmap(ctx->workpage);
	ablkcipher_request_free(req);
	if (res < 0) {
		printk_ratelimited(
		    KERN_ERR "%s: Error in ext4_fname_encrypt (error code %d)\n",
		    __func__, res);
	}
	return res;
}

/**
 * ext4_fname_encrypt_simple() -
 *
 * This function encrypts the input filename, and returns the length of the
 * ciphertext. Errors are returned as negative numbers.  We trust the caller to
 * allocate sufficient memory to oname string.
 */
static int ext4_fname_encrypt(struct ext4_fname_crypto_ctx *ctx,
			      struct ext4_cstr *oname,
			      const struct ext4_cstr *iname)
{
	struct ext4_cstr tmp_in[2], tmp_out[1];
	u32 ciphertext_len, first_output_len, first_input_len,
		num_reused_bytes, input_trailing_bytes, output_trailing_bytes;
	int res;

	if (iname->len <= 0 || iname->len > ctx->lim)
		return -EIO;

	ciphertext_len = ext4_fname_crypto_round_up(iname->len,
		EXT4_CRYPTO_BLOCK_SIZE);
	ciphertext_len = (ciphertext_len > ctx->lim)
		? ctx->lim : ciphertext_len;

	/* Figure out whether special handling in the last block is
	 * needed because the output size is limited to non-integer multiple
	 * of EXT4_CRYPTO_BLOCK_SIZE */
	output_trailing_bytes = ciphertext_len & (EXT4_CRYPTO_BLOCK_SIZE-1);
	first_output_len = ciphertext_len ^ output_trailing_bytes;
	first_input_len = (first_output_len < iname->len)
				? first_output_len : iname->len;
	num_reused_bytes = EXT4_CRYPTO_BLOCK_SIZE - output_trailing_bytes;
	input_trailing_bytes = iname->len - first_input_len;

	tmp_in[0].name = iname->name;
	tmp_in[0].len = first_input_len;
	tmp_out[0].name = oname->name;

	res = ext4_cbc_encrypt(ctx, tmp_out, tmp_in, first_input_len);
	if (res < 0 || output_trailing_bytes == 0) {
		oname->len = res;
		return res;
	}
	BUG_ON(first_output_len != first_input_len);
	BUG_ON(input_trailing_bytes <= 0 || output_trailing_bytes <= 0);
	BUG_ON(input_trailing_bytes >= EXT4_CRYPTO_BLOCK_SIZE
			|| output_trailing_bytes >= EXT4_CRYPTO_BLOCK_SIZE);

	tmp_in[0].name = oname->name + first_output_len-num_reused_bytes;
	tmp_in[0].len = num_reused_bytes;
	tmp_in[1].name = iname->name + first_input_len;
	tmp_in[1].len = input_trailing_bytes;
	tmp_out[0].name = oname->name + first_output_len-num_reused_bytes;
	res = ext4_cbc_encrypt(ctx, tmp_out, tmp_in,
			       num_reused_bytes + input_trailing_bytes);
	if (res < 0)
		return res;

	oname->len = ciphertext_len;
	return ciphertext_len;
}
/*
 * ext4_fname_decrypt()
 *	This function decrypts the input filename, and returns
 *	the length of the plaintext.
 *	Errors are returned as negative numbers.
 *	We trust the caller to allocate sufficient memory to oname string.
 */
int ext4_fname_decrypt(struct ext4_fname_crypto_ctx *ctx,
		       struct ext4_cstr *oname,
		       const struct ext4_cstr *iname)
{
	struct ext4_cstr tmp_in[2], tmp_out[1];
	char *tmp_blk = ctx->tmp_buf;
	u32 input_trailing_bytes, second_input_len, num_reused_bytes;
	int res;

	if (iname->len <= 0 || iname->len > ctx->lim)
		return -EIO;

	input_trailing_bytes = iname->len & (EXT4_CRYPTO_BLOCK_SIZE-1);
	num_reused_bytes = EXT4_CRYPTO_BLOCK_SIZE - input_trailing_bytes;
	second_input_len = iname->len ^ input_trailing_bytes;
	/* We allow odd inputs only if we are running into limit */
	if (input_trailing_bytes != 0 && iname->len != ctx->lim)
		return -EIO;

	if (input_trailing_bytes != 0) {
		/* Perform the last-block dance */
		tmp_in[0].name =
			iname->name+iname->len - EXT4_CRYPTO_BLOCK_SIZE;
		tmp_in[0].len = EXT4_CRYPTO_BLOCK_SIZE;
		tmp_out[0].name = tmp_blk;
		res = ext4_cbc_decrypt(ctx, tmp_out, tmp_in,
				       EXT4_CRYPTO_BLOCK_SIZE);
		if (res < 0)
			return res;
		tmp_in[0].name = iname->name;
		tmp_in[0].len = iname->len-EXT4_CRYPTO_BLOCK_SIZE;
		tmp_in[1].name = tmp_blk;
		tmp_in[1].len = num_reused_bytes;
	} else {
		tmp_in[0].name = iname->name;
		tmp_in[0].len = iname->len;
	}

	tmp_out[0].name = oname->name;

	res = ext4_cbc_decrypt(ctx, tmp_out, tmp_in, second_input_len);
	if (res < 0)
		return res;
	if (input_trailing_bytes != 0) {
		/* Copy the trailing bytes */
		memcpy(oname->name+second_input_len, tmp_blk+num_reused_bytes,
				input_trailing_bytes);
	}
	oname->len = strnlen(oname->name, iname->len);
	return oname->len;
}

/**
 * ext4_fname_encode_digest() -
 *
 * Encodes the input digest using characters from the set [a-zA-Z0-9_+].
 * The encoded string is roughly 4/3 times the size of the input string.
 */
int ext4_fname_encode_digest(char *dst, char *src, u32 len)
{
	static const char *lookup_table =
		"abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_+";
	u32 current_chunk, num_chunks, i;
	char tmp_buf[3];
	u32 c0, c1, c2, c3;

	current_chunk = 0;
	num_chunks = len/3;
	for (i = 0; i < num_chunks; i++) {
		c0 = src[3*i] & 0x3f;
		c1 = (((src[3*i]>>6)&0x3) | ((src[3*i+1] & 0xf)<<2)) & 0x3f;
		c2 = (((src[3*i+1]>>4)&0xf) | ((src[3*i+2] & 0x3)<<4)) & 0x3f;
		c3 = (src[3*i+2]>>2) & 0x3f;
		dst[4*i] = lookup_table[c0];
		dst[4*i+1] = lookup_table[c1];
		dst[4*i+2] = lookup_table[c2];
		dst[4*i+3] = lookup_table[c3];
	}
	if (i*3 < len) {
		memset(tmp_buf, 0, 3);
		memcpy(tmp_buf, &src[3*i], len-3*i);
		c0 = tmp_buf[0] & 0x3f;
		c1 = (((tmp_buf[0]>>6)&0x3) | ((tmp_buf[1] & 0xf)<<2)) & 0x3f;
		c2 = (((tmp_buf[1]>>4)&0xf) | ((tmp_buf[2] & 0x3)<<4)) & 0x3f;
		c3 = (tmp_buf[2]>>2) & 0x3f;
		dst[4*i] = lookup_table[c0];
		dst[4*i+1] = lookup_table[c1];
		dst[4*i+2] = lookup_table[c2];
		dst[4*i+3] = lookup_table[c3];
	}
	return 4*i;
}

/**
 * ext4_fname_hash() -
 *
 * This function computes the hash of the input filename, and sets the output
 * buffer to the *encoded* digest.  It returns the length of the digest as its
 * return value.  Errors are returned as negative numbers.  We trust the caller
 * to allocate sufficient memory to oname string.
 */
static int ext4_fname_hash(struct ext4_fname_crypto_ctx *ctx,
			   struct ext4_cstr *oname,
			   const struct ext4_cstr *iname)
{
	struct scatterlist src[EXT4_FNAME_NUM_SCATTER_ENTRIES];
	struct ext4_fname_crypto_buf_desc buf_desc[1];

	struct hash_desc desc = {
		.tfm = (struct crypto_hash *)ctx->htfm,
		.flags = CRYPTO_TFM_REQ_MAY_SLEEP
	};
	int res = 0;

	buf_desc[0].buf = iname->name;
	buf_desc[0].size = iname->len;
	res = map_bufs_to_sg(src, EXT4_FNAME_NUM_SCATTER_ENTRIES,
		buf_desc, 1);
	if (res < 0)
		goto out;
	res = crypto_hash_init(&desc);
	if (res) {
		printk(KERN_ERR
		       "%s: Error initializing crypto hash; res = [%d]\n",
		       __func__, res);
		goto out;
	}
	res = crypto_hash_update(&desc, src, iname->len);
	if (res) {
		printk(KERN_ERR
		       "%s: Error updating crypto hash; res = [%d]\n",
		       __func__, res);
		goto out;
	}
	res = crypto_hash_final(&desc,
		&oname->name[EXT4_FNAME_CRYPTO_DIGEST_SIZE]);
	if (res) {
		printk(KERN_ERR
		       "%s: Error finalizing crypto hash; res = [%d]\n",
		       __func__, res);
		goto out;
	}
	/* Encode the digest as a printable string--this will increase the
	 * size of the digest */
	res = ext4_fname_encode_digest(oname->name,
		&oname->name[EXT4_FNAME_CRYPTO_DIGEST_SIZE],
		EXT4_FNAME_CRYPTO_DIGEST_SIZE);
	oname->len = res;
out:
	return res;
}

/**
 * ext4_free_fname_crypto_ctx() -
 *
 * Frees up a crypto context.
 */
void ext4_free_fname_crypto_ctx(struct ext4_fname_crypto_ctx *ctx)
{
	if (ctx == NULL || IS_ERR(ctx))
		return;

	if (ctx->ctfm) {
		if (!IS_ERR(ctx->ctfm)) {
			crypto_free_ablkcipher(ctx->ctfm);
		}
	}
	if (ctx->htfm) {
		if (!IS_ERR(ctx->htfm)) {
			crypto_free_hash(ctx->htfm);
		}
	}
	if (ctx->workpage) {
		if (!IS_ERR(ctx->workpage)) {
			__free_page(ctx->workpage);
		}
	}
	kfree(ctx);
}

/**
 * ext4_put_fname_crypto_ctx() -
 *
 * Return: The crypto context onto free list. If the free list is above a
 * threshold, completely frees up the context, and returns the memory.
 *
 * TODO: Currently we directly free the crypto context. Eventually we should
 * add code it to return to free list. Such an approach will increase
 * efficiency of directory lookup.
 */
void ext4_put_fname_crypto_ctx(struct ext4_fname_crypto_ctx **ctx)
{
	if (*ctx == NULL || IS_ERR(*ctx))
		return;
	ext4_free_fname_crypto_ctx(*ctx);
	*ctx = NULL;
}

/**
 * ext4_search_fname_crypto_ctx() -
 */
static struct ext4_fname_crypto_ctx *ext4_search_fname_crypto_ctx(
		const struct ext4_encryption_key *key)
{
	return NULL;
}

/**
 * ext4_alloc_fname_crypto_ctx() -
 */
struct ext4_fname_crypto_ctx *ext4_alloc_fname_crypto_ctx(
	const struct ext4_encryption_key *key)
{
	struct ext4_fname_crypto_ctx *ctx;

	ctx = kmalloc(sizeof(struct ext4_fname_crypto_ctx), GFP_NOFS);
	if (ctx == NULL)
		return ERR_PTR(-ENOMEM);
	if (key->mode == EXT4_ENCRYPTION_MODE_INVALID) {
		/* This will automatically set key mode to invalid
		 * As enum for ENCRYPTION_MODE_INVALID is zero */
		memset(&ctx->key, 0, sizeof(ctx->key));
	} else {
		memcpy(&ctx->key, key, sizeof(struct ext4_encryption_key));
	}
	ctx->has_valid_key = (EXT4_ENCRYPTION_MODE_INVALID == key->mode)
		? 0 : 1;
	ctx->ctfm_key_is_ready = 0;
	ctx->ctfm = NULL;
	ctx->htfm = NULL;
	ctx->workpage = NULL;
	return ctx;
}

/**
 * ext4_get_fname_crypto_ctx() -
 *
 * Allocates a free crypto context and initializes it to hold
 * the crypto material for the inode.
 *
 * Return: NULL if not encrypted. Error value on error. Valid pointer otherwise.
 */
struct ext4_fname_crypto_ctx *ext4_get_fname_crypto_ctx(
	struct inode *inode, u32 max_ciphertext_len)
{
	struct ext4_fname_crypto_ctx *ctx;
	struct ext4_inode_info *ei = EXT4_I(inode);
	int res;

	/* Read the encryption key from XATTR, decrypt it, and store it into
	 * inode info */
	res = ext4_get_crypto_key_inode(inode);
	if (res)
		return NULL;

	/* If the directory/symlink is unencrypted, return null crypto
	 * context */
	if (EXT4_ENCRYPTION_MODE_PLAINTEXT == ei->i_encryption_key.mode)
		return NULL;

	/* Get a crypto context based on the key.
	 * A new context is allocated if no context matches the requested key.
	 */
	ctx = ext4_search_fname_crypto_ctx(&(ei->i_encryption_key));
	if (ctx == NULL)
		ctx = ext4_alloc_fname_crypto_ctx(&(ei->i_encryption_key));
	if (IS_ERR(ctx))
		return ctx;

	if (ctx->has_valid_key) {
		/* As a first cut, we will allocate new tfm in every call.
		 * later, we will keep the tfm around, in case the key gets
		 * re-used */
		if (ctx->ctfm == NULL) {
			ctx->ctfm = crypto_alloc_ablkcipher("ecb(aes)",
					0, 0);
		}
		if (IS_ERR(ctx->ctfm)) {
			res = PTR_ERR(ctx->ctfm);
			printk(
			    KERN_DEBUG "%s: error (%d) allocating crypto tfm\n",
			    __func__, res);
			ctx->ctfm = NULL;
			ext4_put_fname_crypto_ctx(&ctx);
			return ERR_PTR(res);
		}
		if (ctx->ctfm == NULL) {
			printk(
			    KERN_DEBUG "%s: could not allocate crypto tfm\n",
			    __func__);
			ext4_put_fname_crypto_ctx(&ctx);
			return ERR_PTR(-ENOMEM);
		}
		if (ctx->workpage == NULL) {
			ctx->workpage = alloc_page(GFP_NOFS);
		}
		if (IS_ERR(ctx->workpage)) {
			res = PTR_ERR(ctx->workpage);
			printk(
			    KERN_DEBUG "%s: error (%d) allocating work page\n",
			    __func__, res);
			ctx->workpage = NULL;
			ext4_put_fname_crypto_ctx(&ctx);
			return ERR_PTR(res);
		}
		if (ctx->workpage == NULL) {
			printk(
			    KERN_DEBUG "%s: could not allocate work page\n",
			    __func__);
			ext4_put_fname_crypto_ctx(&ctx);
			return ERR_PTR(-ENOMEM);
		}
		ctx->lim = max_ciphertext_len;
		BUG_ON(ctx->key.mode != EXT4_ENCRYPTION_MODE_AES_256_CBC);
		crypto_ablkcipher_clear_flags(ctx->ctfm, ~0);
		crypto_tfm_set_flags(crypto_ablkcipher_tfm(ctx->ctfm),
			CRYPTO_TFM_REQ_WEAK_KEY);

		/* If we are lucky, we will get a context that is already
		 * set up with the right key. Else, we will have to
		 * set the key */
		if (!ctx->ctfm_key_is_ready) {
			/* Since our crypto objectives for filename encryption
			 * are pretty weak,
			 * we directly use the inode master key */
			res = crypto_ablkcipher_setkey(ctx->ctfm,
					ctx->key.raw, ctx->key.size);
			if (res) {
				ext4_put_fname_crypto_ctx(&ctx);
				return ERR_PTR(-EIO);
			}
			ctx->ctfm_key_is_ready = 1;
		} else {
			/* In the current implementation, key should never be
			 * marked "ready" for a context that has just been
			 * allocated. So we should never reach here */
			 BUG();
		}
	}
	if (ctx->htfm == NULL) {
		ctx->htfm = crypto_alloc_hash("sha256", 0, CRYPTO_ALG_ASYNC);
	}
	if (IS_ERR(ctx->htfm)) {
		res = PTR_ERR(ctx->htfm);
		printk(KERN_DEBUG "%s: error (%d) allocating hash tfm\n",
			__func__, res);
		ctx->htfm = NULL;
		ext4_put_fname_crypto_ctx(&ctx);
		return ERR_PTR(res);
	}
	if (ctx->htfm == NULL) {
		printk(KERN_DEBUG "%s: could not allocate hash tfm\n",
				__func__);
		ext4_put_fname_crypto_ctx(&ctx);
		return ERR_PTR(-ENOMEM);
	}

	return ctx;
}

/**
 * ext4_fname_crypto_round_up() -
 *
 * Return: The next multiple of block size
 */
u32 ext4_fname_crypto_round_up(u32 size, u32 blksize)
{
	return ((size+blksize-1)/blksize)*blksize;
}

/**
 * ext4_fname_crypto_namelen_on_disk() -
 */
int ext4_fname_crypto_namelen_on_disk(struct ext4_fname_crypto_ctx *ctx,
				      u32 namelen)
{
	u32 ciphertext_len;

	if (ctx == NULL)
		return -EIO;
	if (!(ctx->has_valid_key))
		return -EACCES;
	ciphertext_len = ext4_fname_crypto_round_up(namelen,
						    EXT4_CRYPTO_BLOCK_SIZE);
	ciphertext_len = (ciphertext_len > ctx->lim)
		? ctx->lim : ciphertext_len;
	return (int) ciphertext_len;
}

/**
 * ext4_fname_crypto_alloc_obuff() -
 *
 * Allocates an output buffer that is sufficient for the crypto operation
 * specified by the context and the direction.
 */
int ext4_fname_crypto_alloc_buffer(struct ext4_fname_crypto_ctx *ctx,
				   unsigned char **obuf, u32 *olen, u32 ilen)
{
	if (!ctx)
		return -EIO;
	*olen = ext4_fname_crypto_round_up(ilen, EXT4_CRYPTO_BLOCK_SIZE);
	if (*olen < EXT4_FNAME_CRYPTO_DIGEST_SIZE*2)
		*olen = EXT4_FNAME_CRYPTO_DIGEST_SIZE*2;
	/* Allocated buffer can hold one more character to null-terminate the
	 * string */
	*obuf = kmalloc(*olen, GFP_NOFS);
	if (!(*obuf))
		return -ENOMEM;
	return 0;
}

/**
 * ext4_fname_crypto_free_buffer() -
 *
 * Frees the buffer allocated for crypto operation.
 * NOTE: This function is called from a loop defined in
 * ext4_fname_crypto_free_buffer() macro.
 */
void ext4_fname_crypto_free_buffer(void **buf)
{
	if (*buf == NULL || IS_ERR(buf))
		return;
	kfree(*buf);
	*buf = NULL;
}

/**
 * ext4_fname_disk_to_usr() - converts a filename from disk space to user space
 */
int ext4_fname_disk_to_usr(struct ext4_fname_crypto_ctx *ctx,
			   struct ext4_cstr *oname,
			   const struct ext4_cstr *iname)
{
	if (ctx == NULL)
		return -EIO;
	if (iname->len < 3) {
		/*Check for . and .. */
		if (iname->name[0] == '.' && iname->name[iname->len-1] == '.') {
			oname->name[0] = '.';
			oname->name[iname->len-1] = '.';
			oname->len = iname->len;
			return oname->len;
		}
	}
	if (ctx->has_valid_key)
		return ext4_fname_decrypt(ctx, oname, iname);
	else
		return ext4_fname_hash(ctx, oname, iname);
}

/**
 * ext4_fname_usr_to_disk() - converts a filename from user space to disk space
 */
int ext4_fname_usr_to_disk(struct ext4_fname_crypto_ctx *ctx,
			   struct ext4_cstr *oname,
			   const struct ext4_cstr *iname)
{
	int res;

	if (ctx == NULL)
		return -EIO;
	if (iname->len < 3) {
		/*Check for . and .. */
		if (iname->name[0] == '.' &&
				iname->name[iname->len-1] == '.') {
			oname->name[0] = '.';
			oname->name[iname->len-1] = '.';
			oname->len = iname->len;
			return oname->len;
		}
	}
	if (ctx->has_valid_key) {
		res = ext4_fname_encrypt(ctx, oname, iname);
		return res;
	}
	/* Without a proper key, a user is not allowed to modify the filenames
	 * in a directory. Consequently, a user space name cannot be mapped to
	 * a disk-space name */
	return -EACCES;
}

/**
 * ext4_fname_usr_to_htree() - converts a filename from user space to htree-access string
 */
int ext4_fname_usr_to_htree(struct ext4_fname_crypto_ctx *ctx,
			    struct ext4_cstr *oname,
			    const struct ext4_cstr *iname)
{
	struct ext4_cstr tmp = {
		.name = NULL,
		.len = 0
	};
	int res = 0;

	if (ctx == NULL)
		return -EIO;

	if (iname->len < 3) {
		/*Check for . and .. */
		if (iname->name[0] == '.' &&
				iname->name[iname->len-1] == '.') {
			oname->name[0] = '.';
			oname->name[iname->len-1] = '.';
			oname->len = iname->len;
			return oname->len;
		}
	}
	if (ctx->has_valid_key) {
		/* First encrypt the plaintext name */
		res = ext4_fname_crypto_alloc_buffer(ctx, &tmp.name,
			&tmp.len, iname->len);
		if (res < 0)
			goto out;
		res = ext4_fname_encrypt(ctx, &tmp, iname);
		if (res < 0)
			goto out;
		/* Now compute the hash */
		res = ext4_fname_hash(ctx, oname, &tmp);
		goto out;

	} else {
		/* Without a proper key, the user space name is used as is to
		 * access the htree */
		memcpy(oname->name, iname->name, iname->len);
		res = iname->len;
	}

out:
	ext4_fname_crypto_free_buffer((void **)&tmp.name);
	return res;
}

/**
 * ext4_fname_disk_to_htree() - converts a filename from disk space to htree-access string
 */
int ext4_fname_disk_to_htree(struct ext4_fname_crypto_ctx *ctx,
	struct ext4_cstr *oname, const struct ext4_cstr *iname) {
	if (ctx == NULL)
		return -EIO;

	if (iname->len < 3) {
		/*Check for . and .. */
		if (iname->name[0] == '.' &&
				iname->name[iname->len-1] == '.') {
			oname->name[0] = '.';
			oname->name[iname->len-1] = '.';
			oname->len = iname->len;
			return oname->len;
		}
	}
	return ext4_fname_hash(ctx, oname, iname);
}
