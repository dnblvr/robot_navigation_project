

#include "../inc/I2C.h"


void config_EUSCI_B1_I2C(void) {

    // hold the eUSC module in reset mode (disable module)
    EUSCI_B1->CTLW0 = 0x0001;

    // master mode, I2C, synchronous, SMCLK, no ACK on slave address, receiver
    EUSCI_B1->CTLW0 = 0x0F81;


    // 12 MHz/120 = 100 KHz baud clock
    EUSCI_B1->BRW   = 120;

    // I2C function on P6.5 = SCL and P6.4 = SDA
    P6->SEL0 |=  0x30;
    P6->SEL1 &= ~0x30;

    // enable the eUSC module
    EUSCI_B1->CTLW0 &= ~0x0001;

    // disarm interrupt on the eUSC module
    EUSCI_B1->IE = 0x0;

}


// I2C transmit example
uint8_t I2C_send_byte(
        uint8_t  slave,
        uint8_t *data)
{

    uint8_t status  = 255,
            n       = 0;

    // BBUSY = 1 if I2C bus is busy?
    while (EUSCI_B1->STATW & 0x10) {};

    // hold eUSCI module in reset mode
    EUSCI_B1->CTLW0 |= 0x0001;

    // generate stop condition after 1 byte
    EUSCI_B1->TBCNT  =  1;

    // enable eUSCI module
    EUSCI_B1->CTLW0 &= ~0x0001;

    // I2CCSA[6:0] is slave address
    EUSCI_B1->I2CSA  = slave;

    // UCTXSTP=0 (TX stop), UCTXSTT=1 (TX start), UCTR=1; configured as TX
    EUSCI_B1->CTLW0  = (EUSCI_B1->CTLW0 & (~0x0004)|0x0012);


    // wait for slave address sent
    while (EUSCI_B1->CTLW0 & 0x0002 == 1);


    // TXBUF[7:0] is data
    EUSCI_B1->TXBUF = (uint8_t)(*(data + n));

    // wait for first byte of data to be sent.
    while ((EUSCI_B1->IFG & 0x0002) == 0) {

        // check NACKIFG: no slave acknowledged data
        if (EUSCI_B1->IFG & 0x0030) {
            status = EUSCI_B1->IFG;
            config_EUSCI_B1_I2C();
            return status; // might have to change to return 255
        }
    }

    // return good status
    return 0;
}


// I2C transmit example
uint8_t I2C_send_bytes(
        uint8_t  slave,
        uint8_t *data,
        uint8_t  num_bytes)
{

    uint8_t status = 255,
            n;

    // BBUSY = 1 if I2C bus is busy?
    while (EUSCI_B1->STATW & 0x10) {};

    // hold eUSCI module in reset mode
    EUSCI_B1->CTLW0 |= 0x0001;

    // generate stop condition after 1 byte
    EUSCI_B1->TBCNT  =  num_bytes;

    // enable eUSCI module
    EUSCI_B1->CTLW0 &= ~0x0001;

    // I2CCSA[6:0] is slave address
    EUSCI_B1->I2CSA  = slave;

    // UCTXSTP=0 (TX stop), UCTXSTT=1 (TX start), UCTR=1; configured as TX
    EUSCI_B1->CTLW0  = (EUSCI_B1->CTLW0 & (~0x0004)|0x0012);


    // wait for slave address sent
    while (EUSCI_B1->CTLW0 & 0x0002 == 1);

    for (n = 0; n < num_bytes; n++) {

        // TXBUF[7:0] is data
        EUSCI_B1->TXBUF = (uint8_t)(*(data + n));

        // wait for first byte of data to be sent.
        while ((EUSCI_B1->IFG & 0x0002) == 0) {

            // check NACKIFG: no slave acknowledged data
            if (EUSCI_B1->IFG & 0x0030) {
                status = EUSCI_B1->IFG;
                config_EUSCI_B1_I2C();
                return status; // might have to change to return 255
            }
        }
    }   // for-loop end

    // return good status
    return 0;
}



// I2C receive example
uint8_t I2C_receive_byte(
        uint8_t  slave,
        uint8_t *data)
{

    // BBUSY = 1 if I2C bus is busy?
    while (EUSCI_B1->STATW & 0x10);

    // hold eUSCI module in reset mode
    EUSCI_B1->CTLW0 |=  0x0001;

    // generate stop condition after 1 byte
    EUSCI_B1->TBCNT  =  1;

    // enable eUSCI module
    EUSCI_B1->CTLW0 &= ~0x0001;

    // I2CCSA[6:0] is slave address
//    EUSCI_B1->I2CSA  =  slave & 0x7F;
    EUSCI_B1->I2CSA  =  slave;

    // UCTXSTP=0 (TX stop), UCTXSTT=1 (TX start), UCTR=0
    EUSCI_B1->CTLW0  =  (EUSCI_B1->CTLW0 & (~0x0014)|0x0002);


    //  this might be incorrect. <---- pls fix

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    // character received
    while ((EUSCI_B1->IFG & 0x0001) == 0) {

        // check NACKIFG: no acknowledgement condition
        if (EUSCI_B1->IFG & 0x0030) {

            // check ALIFG: arbitration lost (another master)
            config_EUSCI_B1_I2C();

            return 0xFF;    // return bad status
        }
    }

    // get the reply
    *data = (uint8_t)(EUSCI_B1->RXBUF & 0xFF);

    // return good status
    return 0;

}


uint8_t I2C_receive_bytes(
        uint8_t slave,
        uint8_t *data,
        uint8_t num_bytes)
{
    uint8_t n;

    // BBUSY = 1 if I2C bus is busy?
    while (EUSCI_B1->STATW & 0x10);


    // hold eUSCI module in reset mode
    EUSCI_B1->CTLW0 |=  0x0001;


    // generate stop condition after the set number of bytes
    EUSCI_B1->TBCNT  =  num_bytes;


    // enable eUSCI module
    EUSCI_B1->CTLW0 &= ~0x0001;


    // I2CCSA[6:0] is slave address
//    EUSCI_B1->I2CSA  =  slave & 0x7F;
    EUSCI_B1->I2CSA  =  slave;


    // UCTXSTP=0 (TX stop), UCTXSTT=1 (TX start), UCTR=0
    EUSCI_B1->CTLW0  =  (EUSCI_B1->CTLW0 & ~0x0014|0x0002);


    for (n = 0; n < num_bytes; n++) {

        // waits until a byte of data has been received
        while ((EUSCI_B1->IFG & 0x0001) == 0) {

            // check NACKIFG: no acknowledgement condition
            if (EUSCI_B1->IFG & 0x0030) {

                // check ALIFG: arbitration lost (another master)
                config_EUSCI_B1_I2C();

                return 0xFF;    // return bad status
            }
        }

        // get the reply
        *(data + n) = (uint8_t)(EUSCI_B1->RXBUF & 0xFF);
    }

    // return good status
    return 0;

}

