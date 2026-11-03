/**
 * Текстовые команды на USART2 (115200 8N1).
 * Строка заканчивается CR или LF. HELP печатает все команды.
 */
#ifndef APP_CLI_H
#define APP_CLI_H

void APP_CLI_UartIrq(void);

#endif /* APP_CLI_H */
