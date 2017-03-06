#ifndef _CTA_KEYPAD_H
#define _CTA_KEYPAD_H

#include <linux/types.h>

#define MATRIX_MAX_ROWS		3	
#define MATRIX_MAX_COLS		3

#define KEY(row, col, val)  ((((row) & 0xff) << 24) |\
                 (((col) & 0xff) << 16) |\
                 (val & 0xffff))

#define KEY_ROW(k)		(((k) >> 24) & 0xff)
#define KEY_COL(k)		(((k) >> 16) & 0xff)
#define KEY_VAL(k)		((k) & 0xffff)

struct cta_keymap_data {
	const uint32_t *keymap;
	unsigned int	keymap_size;
};

struct cta_keypad_platform_data {
	const struct cta_keymap_data *keymap_data;

	const unsigned int *row_gpios;
	const unsigned int *col_gpios;

	unsigned int	num_row_gpios;
	unsigned int	num_col_gpios;

	unsigned int	col_scan_delay_us;

	bool		active_low;
	bool		wakeup; //
};


struct cta_ana_keypad_platform_data {
	const struct cta_keymap_data *keymap_data;
	bool		wakeup;
};

#endif /* _CTA_KEYPAD_H */
