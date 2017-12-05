/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __Q6_ASM_H__
#define __Q6_ASM_H__

#define MAX_SESSIONS	16

typedef void (*app_cb) (uint32_t opcode, uint32_t token,
			uint32_t *payload, void *priv);
struct audio_client;
struct audio_client *q6asm_audio_client_alloc(struct device *dev,
					      app_cb cb, void *priv);
void q6asm_audio_client_free(struct audio_client *ac);
int q6asm_get_session_id(struct audio_client *ac);
#endif /* __Q6_ASM_H__ */
