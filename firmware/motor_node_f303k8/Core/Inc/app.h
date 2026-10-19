#ifndef APP_H
#define APP_H

/* 周辺回路の初期化(MX_*_Init)のあとに 1 回呼ぶ */
void app_init(void);

/* メインループで呼び続ける */
void app_loop(void);

#endif /* APP_H */
