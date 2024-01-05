extern UART_HandleTypeDef huart1;

void print_string(const char *str) {
  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 1000);
}

void print_uint16(uint16_t number) {
  char num_buffer[6];
  uint8_t i = 0;

  if (number == 0) {
    print_string("0");
    return;
  }

  while (number > 0) {
    num_buffer[i++] = '0' + (number % 10);
    number /= 10;
  }

  // Reverse the string
  for (uint8_t j = 0; j < i / 2; j++) {
    char temp = num_buffer[j];
    num_buffer[j] = num_buffer[i - j - 1];
    num_buffer[i - j - 1] = temp;
  }

  num_buffer[i] = '\0';
  print_string(num_buffer);
}

void print_uint32(uint32_t number) {
  char num_buffer[11]; // Buffer size increased to accommodate uint32_t
  uint8_t i = 0;

  if (number == 0) {
    print_string("0");
    return;
  }

  while (number > 0) {
    num_buffer[i++] = '0' + (number % 10);
    number /= 10;
  }

  // Reverse the string
  for (uint8_t j = 0; j < i / 2; j++) {
    char temp = num_buffer[j];
    num_buffer[j] = num_buffer[i - j - 1];
    num_buffer[i - j - 1] = temp;
  }

  num_buffer[i] = '\0';
  print_string(num_buffer);
}

void print_int16(int16_t number) {
  char num_buffer[7];
  uint8_t i = 0;

  if (number == 0) {
    print_string("0");
    return;
  }

  if (number < 0) {
    print_string("-");
    number = -number;
  }

  while (number > 0) {
    num_buffer[i++] = '0' + (number % 10);
    number /= 10;
  }

  // Reverse the string
  for (uint8_t j = 0; j < i / 2; j++) {
    char temp = num_buffer[j];
    num_buffer[j] = num_buffer[i - j - 1];
    num_buffer[i - j - 1] = temp;
  }

  num_buffer[i] = '\0';
  print_string(num_buffer);
}

void print_number(uint32_t number) {
  char num_buffer[12];
  uint8_t i = 0;

  if (number == 0) {
    print_string("0");
    return;
  }

  while (number > 0) {
    num_buffer[i++] = '0' + (number % 10);
    number /= 10;
  }

  // Reverse the string
  for (uint8_t j = 0; j < i / 2; j++) {
    char temp = num_buffer[j];
    num_buffer[j] = num_buffer[i - j - 1];
    num_buffer[i - j - 1] = temp;
  }

  num_buffer[i] = '\0';
  print_string(num_buffer);
}

void print_uint64(uint64_t number) {
  char num_buffer[21]; // Large enough buffer for a 64-bit unsigned integer (20 digits + null terminator)
  uint8_t i = 0;

  if (number == 0) {
    print_string("0");
    return;
  }

  while (number > 0) {
    num_buffer[i++] = '0' + (number % 10);
    number /= 10;
  }

  // Reverse the string
  for (uint8_t j = 0; j < i / 2; j++) {
    char temp = num_buffer[j];
    num_buffer[j] = num_buffer[i - j - 1];
    num_buffer[i - j - 1] = temp;
  }

  num_buffer[i] = '\0';
  print_string(num_buffer);
}

void print_float(float number) {
  int32_t int_part = (int32_t)number;
  uint32_t frac_part = (uint32_t)((number - int_part) * 100);

  if (number < 0) {
    print_string("-");
    int_part = -int_part;
    frac_part = -frac_part;
  }

  print_uint32(int_part);
  print_string(".");
  if (frac_part < 10) {
    print_string("0");
  }
  print_uint32(frac_part);
}