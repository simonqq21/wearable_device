#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

uint16_t cobs_encode(char *src, char *dst, uint16_t len);
uint16_t cobs_decode(char *src, char *dst, uint16_t len);
#define NUM_RAW_BYTES 12

int main()
{
    char raw_bytes1[NUM_RAW_BYTES] = {
        0x0,
        0x1,
        0x2,
        0x3,
        0x4,
        0x0,
        0x5,
        0x6,
        0x0,
        0x0,
        0xa,
        0x8,
    };
    /*
    raw bytes: 0 1 2 3 4 0 5 6 0 0 a 8 0
    encoded COBS: 1 5 1 2 3 4 3 5 6 1 3 a 8  0
                    0 1 2 3 4 0 5 6 0 0 a 8  0
    decoded COBS: 0 1 2 3 4 0 5 6 0 0 a 8  0
    */
    char cobs_decoded[16];
    char cobs_encoded[16];

    printf("raw_bytes1 = ");
    for (int i = 0; i < NUM_RAW_BYTES; i++)
    {
        printf("%x ", raw_bytes1[i]);
    }
    printf("\nNUM_RAW_BYTES = %d\n", NUM_RAW_BYTES);

    uint16_t cobs_encoded_length = cobs_encode(raw_bytes1, cobs_encoded, NUM_RAW_BYTES);
    printf("cobs_encoded = ");
    for (int i = 0; i < cobs_encoded_length; i++)
    {
        printf("%x ", cobs_encoded[i]);
    }
    printf("\ncobs_encoded_length = %d\n", cobs_encoded_length);

    uint16_t cobs_decoded_length = cobs_decode(cobs_encoded, cobs_decoded, cobs_encoded_length);
    printf("cobs_decoded = ");
    for (int i = 0; i < cobs_decoded_length; i++)
    {
        printf("%x ", cobs_decoded[i]);
    }
    printf("\ncobs_decoded_length = %d\n", cobs_decoded_length);

    return 0;
}

/**
 * @brief COBS (consistent overhead byte stuffing) encoding function
 *
 * @param src pointer to source unencoded byte sequence
 * @param dst pointer to destination memory for the COBS stuffed byte sequence
 * @param len length of unencoded byte sequence
 * @return length of COBS encoded byte sequence
 */
uint16_t cobs_encode(char *src, char *dst, uint16_t len)
{
    size_t read_index, write_index, code_index;
    /* read index */
    read_index = 0;
    /* write index
    write_index starts at 1 because the first byte is the number of
     bytes to the first 0x0 */
    write_index = 1;
    /* code index in the dst buffer
    it starts at zero and is set to
    indices where 0x0 is read. */
    code_index = 0;
    /* code holds the length from the current
    write pointer to where the next 0x0 is in the
    source buffer.
    code starts at 1 because the very first byte in
    a COBS encoded string is the length to the next
    0x0.
    */
    uint8_t code = 1;

    while (read_index < len)
    {
        /* if current read byte is 0x0 */
        if (src[read_index] == 0)
        {
            /* set the code index to the number of bytes
            from the previous 0x0 to the current 0x0 */
            dst[code_index] = code;
            /* move code_index to the next write_index
            code_index gets assigned the value of write_index
            before the increment*/
            code_index = write_index++;
            /* reset number of bytes to the next 0x0 to 1 */
            code = 1;
        }
        /* if current read byte is not 0x0 */
        else
        {
            /* copy the byte to dst */
            dst[write_index++] = src[read_index];
            /* increment number of bytes to the next 0x0 */
            code++;
            /* if code exceeds 0xFF */
            if (code == 0xFF)
            {
                /* set the current code index in dst to 0xFF */
                dst[code_index] = code;
                /* move the code index after the write index */
                code_index = write_index++;
                /* reset number of bytes to the next 0x0 to 1. */
                code = 1;
            }
        }
        read_index++;
    }
    dst[code_index] = code;
    dst[write_index++] = 0x00;
    return write_index;
}

/**
 * @brief COBS (consistent overhead byte stuffing) decoding function
 *
 * @param src pointer to source encoded byte sequence
 * @param dst pointer to destination memory for the COBS decoded byte sequence
 * @param len length of encoded byte sequence
 *
 * @return length of COBS decoded byte sequence
 */
uint16_t cobs_decode(char *src, char *dst, uint16_t len)
{
    size_t read_index, write_index;
    /* read index */
    read_index = 0;
    /* write index */
    write_index = 0;
    /* code value */
    uint8_t code = 0;

    // read COBS encoded string
    while (read_index < len)
    {
        /* get code
        if code exceeded 0xFF, just read the next code byte. */
        code = src[read_index];

        /* out of bounds error */
        if (read_index + code > len && code != 1)
        {
            return 0;
        }

        read_index++;
        /* copy all the bytes until the next 0x0 */
        for (int i = 1; i < code; i++)
        {
            dst[write_index++] = src[read_index++];
        }

        /* write the 0x0 once the next 0x0 index has been reached */
        if (code < 0xFF && read_index < len)
        {
            dst[write_index++] = 0x00;
        }
    }
    write_index--;
    return write_index;
}