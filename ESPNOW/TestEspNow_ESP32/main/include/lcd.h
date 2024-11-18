

static esp_err_t lcd_write_byte(uint8_t cmd, bool is_data);


static void lcd_send_cmd(uint8_t cmd);


void lcd_init(void);


void lcd_clear(void);


void lcd_home(void);


void lcd_set_cursor(uint8_t row, uint8_t col);


void lcd_print(const char* str);

void lcd_backlight(bool on);

extern void lcd_task(void *pvParameter);