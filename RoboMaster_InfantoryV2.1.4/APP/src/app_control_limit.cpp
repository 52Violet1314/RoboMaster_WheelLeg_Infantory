#include "app_control_limit.hpp"
void Controler_Limit(Classic_Data_t *pClassicData) {
  if (pClassicData->Contronller_Data.Fw_L > 6.0f) pClassicData->Contronller_Data.Fw_L = 6.0f;
  if (pClassicData->Contronller_Data.Fw_L < -6.0f) pClassicData->Contronller_Data.Fw_L = -6.0f;
  if (pClassicData->Contronller_Data.Fw_R > 6.0f) pClassicData->Contronller_Data.Fw_R = 6.0f;
  if (pClassicData->Contronller_Data.Fw_R < -6.0f) pClassicData->Contronller_Data.Fw_R = -6.0f;
  if (pClassicData->Contronller_Data.T1_L > 20.5f) pClassicData->Contronller_Data.T1_L = 20.5f;
  if (pClassicData->Contronller_Data.T1_L < -20.5f) pClassicData->Contronller_Data.T1_L = -20.5f;
  if (pClassicData->Contronller_Data.T1_R > 20.5f) pClassicData->Contronller_Data.T1_R = 20.5f;
  if (pClassicData->Contronller_Data.T1_R < -20.5f) pClassicData->Contronller_Data.T1_R = -20.5f;
  if (pClassicData->Contronller_Data.T2_L > 20.5f) pClassicData->Contronller_Data.T2_L = 20.5f;
  if (pClassicData->Contronller_Data.T2_L < -20.5f) pClassicData->Contronller_Data.T2_L = -20.5f;
  if (pClassicData->Contronller_Data.T2_R > 20.5f) pClassicData->Contronller_Data.T2_R = 20.5f;
  if (pClassicData->Contronller_Data.T2_R < -20.5f) pClassicData->Contronller_Data.T2_R = -20.5f;
}
