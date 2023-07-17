/*
 * MusicPlayer.h
 *
 *  Created on: 30 de jun de 2023
 *      Author: rafae
 */

#ifndef SRC_MUSICPLAYER_H_
#define SRC_MUSICPLAYER_H_

#include "main.h"

#define DO 367
#define RE 326
#define MI 292
#define FA 275
#define SOL 245
#define LA 218
#define SI 194

#define c 367
#define d 326
#define e 292
#define f 275
#define g 245
#define gS 231
#define a 218
#define aS 211
#define bb 194

#define cH 184
#define cSH 173
#define dH 164
#define dSH 154
#define eH 146
#define fH 138
#define fSH 129
#define gH 122
#define gSH 116
#define aH 109


#define  a3f    208     // 208 Hz
#define  b3f    233     // 233 Hz
#define  b3     247     // 247 Hz
#define  c4     261     // 261 Hz MIDDLE C
#define  c4s    277     // 277 Hz
#define  e4f    311     // 311 Hz
#define  f4     349     // 349 Hz
#define  a4f    415     // 415 Hz
#define  b4f    466     // 466 Hz
#define  b4     493     //  493 Hz
#define  c5     523     // 523 Hz
#define  c5s    554     // 554  Hz
#define  e5f    622     // 622 Hz
#define  f5     698     // 698 Hz
#define  f5s    740     // 740 Hz
#define  a5f    831     // 831 Hz

#define  rest    0


int MusicPlayer(int PresentStatusMusic, int Speed);

#endif /* SRC_MUSICPLAYER_H_ */



