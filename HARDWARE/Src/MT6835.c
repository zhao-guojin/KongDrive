//
// Created by AerialBird on 25-7-4.
//
#include "MT6835.h"

void MT6835_CS(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void SPI_MT6835_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  // ʹ��GPIOCʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);  // ʹ��SPI3ʱ��

    // GPIOC 10,11,12��ʼ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          // ���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        // �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          // ����
    GPIO_Init(GPIOC, &GPIO_InitStructure);                // ��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          // ���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        // �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          // ����
    GPIO_Init(GPIOC, &GPIO_InitStructure);                // ��ʼ��

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);  // PC10����ΪSPI3
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);  // PC11����ΪSPI3
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);  // PC12����ΪSPI3

    // ��λSPI3
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;    // 16λ����
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;  // ͨ��ʹ��7��15
    SPI_Init(SPI3, &SPI_InitStructure);

    SPI_Cmd(SPI3, ENABLE);
    // MT6835_CS_CLR;
    // SPI_MT6835_ReadWriteByte(0x0000);//��������
    // MT6835_CS_SET;

}

//SPI1�ٶ����ú���
//SPI�ٶ�=fAPB2/��Ƶϵ��
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256
//fAPB2ʱ��һ��Ϊ84Mhz��
void SPI_MT6835_SetSpeed(u8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
    SPI3->CR1&=0XFFC7;//λ3-5���㣬�������ò�����
    SPI3->CR1|=SPI_BaudRatePrescaler;	//����SPI1�ٶ�
    SPI_Cmd(SPI3,ENABLE); //ʹ��SPI1
}
//SPI1 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
uint8_t SPI_MT6835_ReadWriteByte(uint8_t TxData)
{
    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET){}//�ȴ���������

    SPI_I2S_SendData(SPI3, TxData); //ͨ������SPIx����һ��byte����

    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET){} //�ȴ�������һ��byte

    return SPI_I2S_ReceiveData(SPI3); //����ͨ��SPIx������յ�����
}


uint16_t MT6835_readSpi(const MT6835_RegName_e regName)
{
    MT6835_CS_CLR
    // 2. ׼��SPI��ȡ����
    uint16_t zerobuff = 0x1111;  // ���ڽ������ݵĿջ�����
    // ���������֣���ģʽ + �Ĵ�����ַ + ������(0)
    uint16_t controlword = (uint16_t)MT6835_buildCtrlWord(MT6835_CtrlMode_Read_register, regName, 0);
    uint16_t recbuff = 0xbeef;  // ���ջ����ʼֵ�����ڴ����⣩
    // 3. ���Ͷ�����
    SPI_MT6835_ReadWriteByte(controlword);
    // 4. Ƭѡ�ź����⴦��ʵ����Ҫ���壩
    // �����ֲ�˵������Ҫ���壬��ʵ��Ӳ����Ҫ
    MT6835_CS_SET
    delay_ms(1);
    MT6835_CS_CLR
    // 5. ִ��SPI�շ�����
    zerobuff=SPI_MT6835_ReadWriteByte(controlword);
    // 6. �ͷ�Ƭѡ�ź�
    MT6835_CS_SET
    // 7. ��֤������Ч��
    // assert(zerobuff != 0xbeef);  // ȷ�����ջ�����������
    // 8. ������Ч����λ
    return(zerobuff & MT6835_DATA_MASK);
}
// /* ����Ƭѡ�ź� */
// /* ִ������ʽд�� */
// void DRV8301_writeSpi(const DRV8301_RegName_e regName,const uint16_t data) {
//     // ����Ƭѡ�ź�
//     MT6835_CS_CLR
//     delay_ms(1);
//
//     // ִ������ʽд��
//     uint16_t controlword = (uint16_t)DRV8301_buildCtrlWord(DRV8301_CtrlMode_Write, regName, data);
//     SPI_ReadWriteByte(controlword);
//     delay_ms(1);
//
//     // ����Ƭѡ�ź�
//     MT6835_CS_SET
//     delay_ms(1);
//
//     return;
// }  // DRV8301_writeSpi()��������


uint8_t ReadMT6835(uint16_t add)
{
    uint8_t pTxData[3] = {0, 0, 0}, pRxData[3] = {0, 0, 0};
    uint16_t timeout = 0xFFFF;

    // ����CS��
    MT6835_CS_SET // ����Ҫ����ʵ��CS���ŵ����˿ں�����

    // ��ϵ�ַ�Ͷ�����
    add = ((add & 0x0FFF) | 0x8000);  // ����READCOMMAND = 0x8000
    pTxData[0] = (uint8_t)(add >> 8);
    pTxData[1] = (uint8_t)(add);
    pTxData[2] = (uint8_t)(add);      // �����ֽڰ�ԭ�߼��ظ�add���ֽ�

    // ����CS��
    MT6835_CS_CLR

    for (int i = 0; i < 3; i++) {
        // �ȴ����ͻ�����Ϊ��
        timeout = 0xFFFF;
        while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET) {
            if (--timeout == 0) break;
        }
        SPI_I2S_SendData(SPI3, pTxData[i]);

        // �ȴ����ջ������ǿ�
        timeout = 0xFFFF;
        while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) {
            if (--timeout == 0) break;
        }
        pRxData[i] = SPI_I2S_ReceiveData(SPI3);
    }

    // ����CS��
    MT6835_CS_SET

    return pRxData[2];
}