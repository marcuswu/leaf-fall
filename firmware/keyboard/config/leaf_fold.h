#ifndef LEAF_FOLD_H
#define LEAF_FOLD_H

// All left half keys
// First row, left to right
#define L_S00 25
#define L_S01 26
#define L_S02 27
#define L_S03 28
#define L_S04 29
#define L_S05 30

// Second row, left to right
#define L_S06 4
#define L_S07 3
#define L_S08 2
#define L_S09 31
#define L_S10 0
#define L_S11 1

// Third row, left to right
#define L_S12 5
#define L_S13 6
#define L_S14 7
#define L_S15 8
#define L_S16 9
#define L_S17 10

// Fourth row, left to right
#define L_S18 14
#define L_S19 13
#define L_S20 12
#define L_S21 11

#define L_MASK ((1 << L_S00) | (1 << L_S01) | (1 << L_S02) | (1 << L_S03) | (1 << L_S04) | (1 << L_S05) | \
                (1 << L_S06) | (1 << L_S07) | (1 << L_S08) | (1 << L_S09) | (1 << L_S10) | (1 << L_S11) | \
                (1 << L_S12) | (1 << L_S13) | (1 << L_S14) | (1 << L_S15) | (1 << L_S16) | (1 << L_S17) | \
                (1 << L_S18) | (1 << L_S19) | (1 << L_S20) | (1 << L_S21))

// All right half keys
// First row, left to right
#define R_S00 15
#define R_S01 16
#define R_S02 17
#define R_S03 18
#define R_S04 19
#define R_S05 20

// Second row, left to right
#define R_S06 9
#define R_S07 10
#define R_S08 14
#define R_S09 8
#define R_S10 7
#define R_S11 6

// Third row, left to right
#define R_S12 5
#define R_S13 4
#define R_S14 3
#define R_S15 2
#define R_S16 1
#define R_S17 0

// Fourth row, left to right
#define R_S18 31
#define R_S19 30
#define R_S20 29
#define R_S21 28

#define R_MASK ((1 << R_S00) | (1 << R_S01) | (1 << R_S02) | (1 << R_S03) | (1 << R_S04) | (1 << R_S05) | \
                (1 << R_S06) | (1 << R_S07) | (1 << R_S08) | (1 << R_S09) | (1 << R_S10) | (1 << R_S11) | \
                (1 << R_S12) | (1 << R_S13) | (1 << R_S14) | (1 << R_S15) | (1 << R_S16) | (1 << R_S17) | \
                (1 << R_S18) | (1 << R_S19) | (1 << R_S20) | (1 << R_S21))

/* Keyboard side configuration */
#ifdef KEYBOARD_RIGHT
#define PIPE_NUMBER 1
#define INPUT_MASK R_MASK
#define S00 R_S00
#define S01 R_S01
#define S02 R_S02
#define S03 R_S03
#define S04 R_S04
#define S05 R_S05
#define S06 R_S06
#define S07 R_S07
#define S08 R_S08
#define S09 R_S09
#define S10 R_S10
#define S11 R_S11
#define S12 R_S12
#define S13 R_S13
#define S14 R_S14
#define S15 R_S15
#define S16 R_S16
#define S17 R_S17
#define S18 R_S18
#define S19 R_S19
#define S20 R_S20
#define S21 R_S21
#else
#define PIPE_NUMBER 0
#define INPUT_MASK L_MASK
#define S00 L_S00
#define S01 L_S01
#define S02 L_S02
#define S03 L_S03
#define S04 L_S04
#define S05 L_S05
#define S06 L_S06
#define S07 L_S07
#define S08 L_S08
#define S09 L_S09
#define S10 L_S10
#define S11 L_S11
#define S12 L_S12
#define S13 L_S13
#define S14 L_S14
#define S15 L_S15
#define S16 L_S16
#define S17 L_S17
#define S18 L_S18
#define S19 L_S19
#define S20 L_S20
#define S21 L_S21
#endif

#endif /* LEAF_FOLD_H */