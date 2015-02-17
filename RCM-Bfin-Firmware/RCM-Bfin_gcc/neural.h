/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  neural.h -
 *
 *  Part of the RCM-BFin GCC v2.0 firmware for the RCM-BFin robot
 *    
 *  Copyright (C) 2015  Engineering^3
 *
 *  Based on the original Surveyor firmware, written and copyrighted
 *  by Surveyor Corp. 
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details (www.gnu.org/licenses)
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
 
/*
 *
 *
 */
 
void nninit_network();
void nninit_pattern();
void nnset_pattern(int);
int f_logic(int);
void nncalculate_network();
void nncalculat_errors();
void nntrain_network(int);
void nndisplay(int);
void nnscale8x8(unsigned char *, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int);
void nnpack8x8(int);

extern int weights[], neurons[], teach_neurons[], nerror[];
extern unsigned char npattern[], nmask[];

#define NUM_INPUT    64
#define NUM_OUTPUT   16
#define NUM_HIDDEN   16    
#define NUM_NPATTERNS 16

// For accessing weight from input 2 to hidden 3 use:
//    W_IN_HIDDEN(2,3)

#define W_IN_HIDDEN(i, h)  weights[i*NUM_HIDDEN + h]
#define W_HIDDEN_OUT(h, o) weights[NUM_INPUT*NUM_HIDDEN + h*NUM_OUTPUT + o]
#define N_IN(i)     neurons[i]
#define N_HIDDEN(h) neurons[NUM_INPUT + h]
#define N_OUT(o)    neurons[NUM_INPUT + NUM_HIDDEN + o]
#define N_TEACH(o)  teach_neurons[o]
#define E_HIDDEN(h) nerror[h]
#define E_OUT(o)    nerror[NUM_HIDDEN + o]


