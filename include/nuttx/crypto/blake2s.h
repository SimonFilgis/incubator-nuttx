/****************************************************************************
 * include/nuttx/crypto/blake2s.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * This code is based on public-domain/CC0 BLAKE2 reference implementation
 * by Samual Neves, at https://github.com/BLAKE2/BLAKE2/tree/master/ref
 * Copyright 2012, Samuel Neves <sneves@dei.uc.pt>
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_CRYPTO_BLAKE2S_H
#define __INCLUDE_NUTTX_CRYPTO_BLAKE2S_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum blake2s_constant
{
  BLAKE2S_BLOCKBYTES = 64,
  BLAKE2S_OUTBYTES = 32,
  BLAKE2S_KEYBYTES = 32,
  BLAKE2S_SALTBYTES = 8,
  BLAKE2S_PERSONALBYTES = 8
};

typedef struct blake2s_state__
{
  uint32_t h[8];
  uint32_t t[2];
  uint32_t f[2];
  size_t buflen;
  size_t outlen;
  uint8_t buf[BLAKE2S_BLOCKBYTES];
} blake2s_state;

typedef struct blake2s_param__
{
  uint8_t digest_length;                        /* 1 */
  uint8_t key_length;                           /* 2 */
  uint8_t fanout;                               /* 3 */
  uint8_t depth;                                /* 4 */
  uint8_t leaf_length[4];                       /* 8 */
  uint8_t node_offset[4];                       /* 12 */
  uint8_t xof_length[2];                        /* 14 */
  uint8_t node_depth;                           /* 15 */
  uint8_t inner_length;                         /* 16 */

  /* uint8_t  reserved[0]; */

  uint8_t salt[BLAKE2S_SALTBYTES];              /* 24 */
  uint8_t personal[BLAKE2S_PERSONALBYTES];      /* 32 */
} blake2s_param;

#ifdef __GNUC__ > 3
#define BLAKE2_UNALIGNED 1
typedef uint32_t uint32_alias_t __attribute__((may_alias));
typedef uint16_t uint16_alias_t __attribute__((may_alias));
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Streaming API */

int blake2s_init(FAR blake2s_state *S, size_t outlen);
int blake2s_init_key(FAR blake2s_state *S, size_t outlen,
                     FAR const void *key,
                     size_t keylen);
int blake2s_init_param(FAR blake2s_state *S,
                       FAR const blake2s_param *P);
int blake2s_update(FAR blake2s_state *S,
                   FAR const void *in, size_t inlen);
int blake2s_final(FAR blake2s_state *S,
                  FAR void *out, size_t outlen);

/* Simple API */

int blake2s(FAR void *out, size_t outlen,
            FAR const void *in, size_t inlen,
            FAR const void *key, size_t keylen);

/****************************************************************************
 * Public Inline Functions
 ****************************************************************************/

static inline uint32_t blake2_load32(FAR const void *src)
{
#if defined(BLAKE2_UNALIGNED) && !defined(CONFIG_ENDIAN_BIG)
  return *(FAR uint32_alias_t *)src;
#elif !defined(CONFIG_ENDIAN_BIG)
  FAR const uint8_t *p = (FAR const uint8_t *)src;
  return ((uint32_t)(p[0]) << 24) |
         ((uint32_t)(p[1]) << 16) |
         ((uint32_t)(p[2]) << 8) |
         ((uint32_t)(p[3]) << 0);
#else
  FAR const uint8_t *p = (FAR const uint8_t *)src;
  return ((uint32_t)(p[0]) << 0) |
         ((uint32_t)(p[1]) << 8) |
         ((uint32_t)(p[2]) << 16) |
         ((uint32_t)(p[3]) << 24);
#endif
}

static inline uint16_t blake2_load16(FAR const void *src)
{
#if defined(BLAKE2_UNALIGNED) && !defined(CONFIG_ENDIAN_BIG)
  return *(FAR uint16_alias_t *)src;
#elif !defined(CONFIG_ENDIAN_BIG)
  const uint8_t *p = (FAR const uint8_t *)src;
  return ((uint16_t)(p[0]) << 8) | ((uint16_t)(p[1]) << 0);
#else
  const uint8_t *p = (FAR const uint8_t *)src;
  return ((uint16_t)(p[0]) << 0) | ((uint16_t)(p[1]) << 8);
#endif
}

static inline void blake2_store16(FAR void *dst, uint16_t w)
{
#if defined(BLAKE2_UNALIGNED) && !defined(CONFIG_ENDIAN_BIG)
  *(FAR uint16_alias_t *)dst = w;
#elif !defined(CONFIG_ENDIAN_BIG)
  FAR uint8_t *p = (FAR uint8_t *)dst;
  p[1] = (uint8_t)w; w >>= 8;
  p[0] = (uint8_t)w;
#else
  FAR uint8_t *p = (FAR uint8_t *)dst;
  p[0] = (uint8_t)w; w >>= 8;
  p[1] = (uint8_t)w;
#endif
}

static inline void blake2_store32(FAR void *dst, uint32_t w)
{
#if defined(BLAKE2_UNALIGNED) && !defined(CONFIG_ENDIAN_BIG)
  *(FAR uint32_alias_t *)dst = w;
#elif !defined(CONFIG_ENDIAN_BIG)
  FAR uint8_t *p = (FAR uint8_t *) dst;
  p[0] = (uint8_t)(w >> 24);
  p[1] = (uint8_t)(w >> 16);
  p[2] = (uint8_t)(w >> 8);
  p[3] = (uint8_t)(w >> 0);
#else
  FAR uint8_t *p = (FAR uint8_t *) dst;
  p[0] = (uint8_t)(w >> 0);
  p[1] = (uint8_t)(w >> 8);
  p[2] = (uint8_t)(w >> 16);
  p[3] = (uint8_t)(w >> 24);
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CRYPTO_BLAKE2S_H */
