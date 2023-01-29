void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void setImpulseUartData(double time, double angle, double reference_moment);
void setMeasureMomentUart(double measure_moment);
void setSinusUartData(double time, double mesureAngle, double sinusAngle);