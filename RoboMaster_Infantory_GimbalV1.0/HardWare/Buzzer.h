#ifndef __BUZZZER_H
#define __BUZZZER_H
#ifdef __cplusplus
extern "C" {
#endif

void Buzzer_Init(void);
void Buzzer_On(uint16_t freq);
void Buzzer_Stop(void);

#ifdef __cplusplus
}
#endif


#endif // !__BUZZZER_H