/* Includes ------------------------------------------------------------------*/
#include "i2c_ex.h"
#include <stdio.h>
#include <string.h>
#include "i2c.h"

#define UNUSED(X) (void) X /* To avoid gcc/g++ warnings */

#define I2C_RECEIVE_BUFFER_LEN 600

__IO uint8_t aReceiveBuffer[I2C_RECEIVE_BUFFER_LEN];
__IO uint8_t tx_buffer[I2C_RECEIVE_BUFFER_LEN];
__IO uint16_t ubReceiveIndex = 0;
uint8_t *pSlaveTransmitBuffer = 0;
volatile uint8_t i2c_addr = 0;
volatile uint16_t tx_buffer_index = 0;
volatile uint16_t tx_len = 0;
volatile uint32_t i2c_timeout_counter = 0;
volatile uint32_t i2c_stop_timeout_flag = 0;
volatile uint32_t i2c_stop_timeout_counter = 0;

void set_i2c_slave_address(uint8_t addr) { i2c_addr = (addr << 1); }

__weak void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len) {
  /* Prevent unused argument(s) compilation warning */
  UNUSED(rx_data);
  UNUSED(len);
}

void i2c2_it_enable(void) {
  LL_I2C_Enable(I2C2);
  LL_I2C_EnableIT_ADDR(I2C2);
  LL_I2C_EnableIT_NACK(I2C2);
  LL_I2C_EnableIT_ERR(I2C2);
  LL_I2C_EnableIT_STOP(I2C2);
}

void i2c2_it_disable(void) {
  LL_I2C_DisableIT_ADDR(I2C2);
  LL_I2C_DisableIT_NACK(I2C2);
  LL_I2C_DisableIT_ERR(I2C2);
  LL_I2C_DisableIT_STOP(I2C2);
}

void Error_Callback(void) {
  // i2c2_it_enable();
  // LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_NACK);
}

void i2c2_set_send_data(uint8_t *tx_ptr, uint16_t len) {
  if (len > I2C_RECEIVE_BUFFER_LEN) {
    len = I2C_RECEIVE_BUFFER_LEN;
  }

  if (len == 0 || tx_ptr == NULL) {
    return;
  }
  memcpy((void *)tx_buffer, tx_ptr, len);
  tx_buffer_index = 0;
  tx_len = len;
}

void Slave_Reception_Callback(void) {
  aReceiveBuffer[ubReceiveIndex++] = LL_I2C_ReceiveData8(I2C2);
}

void Slave_Ready_To_Transmit_Callback(void) {
  LL_I2C_TransmitData8(I2C2, tx_buffer[tx_buffer_index]);
  tx_buffer_index++;
  if (tx_buffer_index >= tx_len) {
    tx_buffer_index = 0;
  }
}

void I2C2_IRQHandler(void) {
  /* USER CODE BEGIN I2C2_IRQn 0 */
  i2c_timeout_counter++;
  if (i2c_timeout_counter > 10000) {
    LL_I2C_DeInit(I2C2);
    LL_I2C_DisableAutoEndMode(I2C2);
    LL_I2C_Disable(I2C2);
    LL_I2C_DisableIT_ADDR(I2C2);
    user_i2c_init();
    i2c2_it_enable();
    i2c_timeout_counter = 0;
  }
  /* Check ADDR flag value in ISR register */
  if (LL_I2C_IsActiveFlag_ADDR(I2C2)) {
    /* Verify the Address Match with the OWN Slave address */
    if (LL_I2C_GetAddressMatchCode(I2C2) == i2c_addr) {
      if (ubReceiveIndex) {
        i2c2_it_disable();
        Slave_Complete_Callback((uint8_t *)aReceiveBuffer, ubReceiveIndex);
        ubReceiveIndex = 0;
        i2c2_it_enable();
      }
      /* Verify the transfer direction, a write direction, Slave enters receiver
       * mode */
      if (LL_I2C_GetTransferDirection(I2C2) == LL_I2C_DIRECTION_WRITE) {
        /* Clear ADDR flag value in ISR register */
        LL_I2C_ClearFlag_ADDR(I2C2);

        /* Enable Receive Interrupt */
        LL_I2C_EnableIT_RX(I2C2);
        i2c_stop_timeout_flag = 1;
      }
      /* Verify the transfer direction, a read direction, Slave enters
         transmitter mode */
      else if (LL_I2C_GetTransferDirection(I2C2) == LL_I2C_DIRECTION_READ) {
        /* Clear ADDR flag value in ISR register */
        LL_I2C_ClearFlag_ADDR(I2C2);

        /* Enable Transmit Interrupt */
        LL_I2C_EnableIT_TX(I2C2);
      } else {
        /* Clear ADDR flag value in ISR register */
        LL_I2C_ClearFlag_ADDR(I2C2);

        /* Call Error function */
        Error_Callback();
      }
    } else {
      /* Clear ADDR flag value in ISR register */
      LL_I2C_ClearFlag_ADDR(I2C2);

      /* Call Error function */
      Error_Callback();
    }
  }
  /* Check NACK flag value in ISR register */
  else if (LL_I2C_IsActiveFlag_NACK(I2C2)) {
    /* End of Transfer */
    LL_I2C_ClearFlag_NACK(I2C2);
  }
  /* Check TXIS flag value in ISR register */
  else if (LL_I2C_IsActiveFlag_TXIS(I2C2)) {
    /* Call function Slave Ready to Transmit Callback */
    Slave_Ready_To_Transmit_Callback();
  }
  /* Check RXNE flag value in ISR register */
  else if (LL_I2C_IsActiveFlag_RXNE(I2C2)) {
    /* Call function Slave Reception Callback */
    Slave_Reception_Callback();
  }
  /* Check STOP flag value in ISR register */
  else if (LL_I2C_IsActiveFlag_STOP(I2C2)) {
    /* End of Transfer */
    LL_I2C_ClearFlag_STOP(I2C2);

    /* Check TXE flag value in ISR register */
    if (!LL_I2C_IsActiveFlag_TXE(I2C2)) {
      /* Flush the TXDR register */
      LL_I2C_ClearFlag_TXE(I2C2);
    }

    i2c2_it_disable();
    /* Call function Slave Complete Callback */
    Slave_Complete_Callback((uint8_t *)aReceiveBuffer, ubReceiveIndex);
    ubReceiveIndex = 0;
    i2c2_it_enable();
    i2c_stop_timeout_flag = 0;
    i2c_stop_timeout_counter = 0;
  }
  /* Check TXE flag value in ISR register */
  else if (!LL_I2C_IsActiveFlag_TXE(I2C2)) {
    /* Do nothing */
    /* This Flag will be set by hardware when the TXDR register is empty */
    /* If needed, use LL_I2C_ClearFlag_TXE() interface to flush the TXDR
     * register  */
  } else {
    /* Call Error function */
    Error_Callback();
  }
  /* USER CODE END I2C2_IRQn 0 */

  /* USER CODE BEGIN I2C2_IRQn 1 */

  /* USER CODE END I2C2_IRQn 1 */
}
