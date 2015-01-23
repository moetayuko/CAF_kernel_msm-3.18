/*
 * linux/fs/ext4/xattr_user.c
 *
 * Handlers for encryption policy extended attributes
 *
 * Written by Ildar Muslukhov, 2014.
 */

#include <linux/fs.h>
#include <linux/string.h>

#include "ext4.h"
#include "ext4_crypto.h"
#include "ext4_jbd2.h"
#include "xattr.h"

static size_t ext4_xattr_encryption_list(struct dentry *dentry, char *list,
					 size_t list_size, const char *name,
					 size_t name_len, int type)
{
	const size_t prefix_len = XATTR_ENCRYPTION_PREFIX_LEN;
	const size_t total_len = prefix_len + name_len + 1;

	if (list && total_len <= list_size) {
		memcpy(list, XATTR_ENCRYPTION_PREFIX, prefix_len);
		memcpy(list + prefix_len, name, name_len);
		list[prefix_len + name_len] = '\0';
	}
	return total_len;
}

static int ext4_xattr_encryption_get(struct dentry *dentry, const char *name,
				     void *buffer, size_t size, int type)
{
	if (strcmp(name, "") == 0)
		return -EINVAL;
	/* Encryption namespace supports only "policy" to be read from
	 * userspace */
	if (strcmp(name, EXT4_XATTR_NAME_ENCRYPTION_POLICY) != 0)
		return -EINVAL;
	return ext4_xattr_get(dentry->d_inode, EXT4_XATTR_INDEX_ENCRYPTION,
			      name, buffer, size);
}

static int ext4_xattr_encryption_set(struct dentry *dentry, const char *name,
				     const void *value, size_t size, int flags,
				     int type)
{
	int res = -EACCES;

	if (strcmp(name, "") == 0)
		return -EINVAL;
	/* Encryption namespace supports only "policy" set through generic
	 * attribute set path. */
	if (strcmp(name, EXT4_XATTR_NAME_ENCRYPTION_POLICY) != 0)
		return -EINVAL;
	if (!ext4_is_encryption_policy_valid(value, size))
		return -EINVAL;
	/* Change of the policy is not allowed for non empty files and
	 * directories. */
	if ((S_ISREG(dentry->d_inode->i_mode) &&
	    dentry->d_inode->i_size == 0) ||
	    (S_ISDIR(dentry->d_inode->i_mode) &&
	    ext4_empty_dir(dentry->d_inode))) {
		res = ext4_xattr_set(dentry->d_inode,
				     EXT4_XATTR_INDEX_ENCRYPTION,
				     name, value, size, flags);
	}

	return res;
}

/*
 * ext4_xattr_encryption_context_set() - stores a wrapped protector in "context"
 * attribute
 *
 * Made as a separate function on purpose, so that setting this value from user
 * space tools is not available through generic set attribute path.
 */
int ext4_xattr_encryption_context_set(struct dentry *dentry, const void *value,
				      size_t size, int flags, int type)
{
	return ext4_xattr_set(dentry->d_inode,
			      EXT4_XATTR_INDEX_ENCRYPTION,
			      EXT4_XATTR_NAME_ENCRYPTION_CONTEXT,
			      value, size, flags);
}

const struct xattr_handler ext4_xattr_encryption_handler = {
	.prefix = XATTR_ENCRYPTION_PREFIX,
	.list   = ext4_xattr_encryption_list,
	.get    = ext4_xattr_encryption_get,
	.set    = ext4_xattr_encryption_set,
};
