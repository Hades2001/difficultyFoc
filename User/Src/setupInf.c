#include "setupInf.h"

setupinf_t setup;

uint8_t initSetupInfo(void)
{
    setup.readFromE2prom = &_readFromE2prom;
    setup.writeToE2prom = &_writeToE2prom;
    return 0;
}

int8_t _readFromE2prom(void)
{
    uint8_t* dataptr = (uint8_t*)&setup.data;
    memset(dataptr,0,sizeof(setupdata_t));
    HAL_StatusTypeDef res = HAL_I2C_Mem_Read(&hi2c2, 0x00a1, 0, I2C_MEMADD_SIZE_8BIT, dataptr, sizeof(setupdata_t), 100);
    if(( setup.data.flag != 0x48618461 )||( res != HAL_OK ))
    {
        setup.data.flag = 0x48618461;
        setup.data.zero_offset = 0.0f;
        setup.data.zero_elec_angle = 0;
        return -2;
    }
    else if( res != HAL_OK )
    {
        setup.data.flag = 0x48618461;
        setup.data.zero_offset = 0.0f;
        setup.data.zero_elec_angle = 0;
        return -1;
    }
    return 0;
}

int8_t _writeToE2prom(void)
{
    uint8_t* dataptr =  (uint8_t*)&setup.data;
    HAL_StatusTypeDef res = HAL_I2C_Mem_Write(&hi2c2, 0x00a0, 0, I2C_MEMADD_SIZE_8BIT, dataptr, sizeof(setupdata_t), 100);
    return ( res == HAL_OK ) ? 0 : -1;
}



