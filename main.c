/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
/** @brief BLE address length. */
#define BLE_GAP_ADDR_LEN (6)
#define SWAP_16(a) a = ((a & 0x00FF) << 8) | ((a & 0xFF00) >> 8);

#define SCAN_ENABLE  0x81 // Включить сканирование
#define SCAN_DISABLE 0x82 // Отключить сканирование
#define TABLE_READ   0x83 // Считать BLE_ID_TABLE
#define TABLE_READ_CONTINUE 0x84 // продолжить чтение BLE_ID_TABLE

#define BLE_ID_TABLE_SIZE 32
#pragma pack ( push, 1 )
struct my_ble_id_table_cell_s
{
    uint8_t addr[BLE_GAP_ADDR_LEN]; // Постоянная составляющая ускорения
    uint8_t m_data_0_dev_type; //DEVICE_TYPE // Наши данные для оповещения
    uint8_t m_data_1_bat_lvl;  // Заряд батареи
    uint8_t m_data_2_rec_flag; // Флаг включения записи
    int8_t  tx_power_level;    // Мощность передатчика
};

struct my_ble_id_table_s
{
    uint16_t	size;
    uint16_t	count;
    struct my_ble_id_table_cell_s cells[BLE_ID_TABLE_SIZE];
};
#pragma pack ( pop )

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev1.0";
static uint32_t mode;
static uint8_t bits = 8;
static uint32_t speed = 1000000;
static uint16_t delay;
static int verbose;
static int iterations;
struct my_ble_id_table_s my_ble_id_table;

uint8_t default_tx[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xF0, 0x0D,
};

uint8_t default_rx[ARRAY_SIZE(default_tx)] = {0, };
char *input_tx;

static void hex_dump(const void *src, size_t length, size_t line_size,
		     char *prefix)
{
	int i = 0;
	const unsigned char *address = src;
	const unsigned char *line = address;
	unsigned char c;

	printf("%s | ", prefix);
	while (length-- > 0) {
		printf("%02X ", *address++);
		if (!(++i % line_size) || (length == 0 && i % line_size)) {
			if (length == 0) {
				while (i++ % line_size)
					printf("__ ");
			}
			printf(" | ");  /* right close */
			while (line < address) {
				c = *line++;
				printf("%c", (c < 33 || c == 255) ? 0x2E : c);
			}
			printf("\n");
			if (length > 0)
				printf("%s | ", prefix);
		}
	}
}

/*
 *  Unescape - process hexadecimal escape character
 *      converts shell input "\x23" -> 0x23
 */
static int unescape(char *_dst, char *_src, size_t len)
{
	int ret = 0;
	int match;
	char *src = _src;
	char *dst = _dst;
	unsigned int ch;

	while (*src) {
		if (*src == '\\' && *(src+1) == 'x') {
			match = sscanf(src + 2, "%2x", &ch);
			if (!match)
				pabort("malformed input string");

			src += 4;
			*dst++ = (unsigned char)ch;
		} else {
			*dst++ = *src++;
		}
		ret++;
	}
	return ret;
}

static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int ret;
/*
	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

	// spi mode
	ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	// bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	// max speed hz
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: 0x%x bits per word: %d max speed: %d kHz\n", mode, bits, speed/1000);
*/
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
		.cs_change = 0,
	};

/*
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = 20,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
*/
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	if (verbose)
		hex_dump(tx, len, 32, "TX");

	if (verbose)
		hex_dump(rx, len, 32, "RX");
//	sleep(0.01);

//	ret = ioctl(fd, SPI_IOC_MESSAGE(0), NULL);
//	if (ret < 1)
//		pabort("can't send spi message");


//	close(fd);
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3vpNR24SI]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word\n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n"
	     "  -v --verbose  Verbose (show tx buffer)\n"
	     "  -p            Send data (e.g. \"1234\\xde\\xad\")\n"
	     "  -N --no-cs    no chip select\n"
	     "  -R --ready    slave pulls low to pause\n"
	     "  -I --iter     iterations\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "no-cs",   0, 0, 'N' },
			{ "ready",   0, 0, 'R' },
			{ "verbose", 0, 0, 'v' },
			{ "iter",    1, 0, 'I' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NRp:vI:",
				lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'N':
			mode |= SPI_NO_CS;
			break;
		case 'v':
			verbose = 1;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		case 'p':
			input_tx = optarg;
			break;
		case 'I':
			iterations = atoi(optarg);
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

static void transfer_escaped_string(int fd, char *str)
{
	size_t size = strlen(str);
	uint8_t *tx;
	uint8_t *rx;

	tx = malloc(size);
	if (!tx)
		pabort("can't allocate tx buffer");

	rx = malloc(size);
	if (!rx)
		pabort("can't allocate rx buffer");

	size = unescape((char *)tx, str, size);
	transfer(fd, tx, rx, size);
	free(rx);
	free(tx);
}
void print_file(void)
{
	FILE* fd2;
	fd2 = fopen("/sys/kernel/debug/gpio", "r");
	if (fd2 < 0)
		pabort("can't open /sys/kernel/debug/gpio");

	int c; // note: int, not char, required to handle EOF
	while ((c = fgetc(fd2)) != EOF) // standard C I/O file reading loop
	putchar(c);

	fclose(fd2);

};
int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;

	parse_opts(argc, argv);

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");


	// spi mode
	//mode = 0;

	ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	// bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	// max speed hz
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: 0x%x bits per word: %d max speed: %d kHz\n", mode, bits, speed/1000);

	if (input_tx){
		transfer_escaped_string(fd, input_tx);
	} else {
		default_tx[0] = TABLE_READ;
		//bits = 8;
		transfer(fd, default_tx, default_rx, 1);
        //print_file();
        sleep(0.01);
        //print_file();
		memset(default_tx, 0x00, 4);
		default_tx[0] = TABLE_READ_CONTINUE;
		//bits = 8;
		transfer(fd, default_tx, (uint8_t *)&my_ble_id_table, 6);
        sleep(0.01);
		SWAP_16(my_ble_id_table.size);
		SWAP_16(my_ble_id_table.count);
		//printf("my_ble_id_table[0,1,2,3]: 0x%2x 0x%2x 0x%2x 0x%2x\n", (uint8_t *)&my_ble_id_table, (uint8_t *)&my_ble_id_table+1, (uint8_t *)&my_ble_id_table+2, (uint8_t *)&my_ble_id_table+3);
		printf("size: %d count: %d\n", my_ble_id_table.size , my_ble_id_table.count);
		if (my_ble_id_table.count > 32) my_ble_id_table.count = 0;
		while (my_ble_id_table.count > 0){
			memset(default_tx, 0x00, 10);
			default_tx[0] = TABLE_READ_CONTINUE;
		    bits = 8;
			transfer(fd, default_tx, (uint8_t *)&my_ble_id_table.cells, 10);
            sleep(0.01);

			printf("ID: %02X:%02X:%02X:%02X:%02X:%02X tx_pow:"\
				, my_ble_id_table.cells[0].addr[5], my_ble_id_table.cells[0].addr[4], my_ble_id_table.cells[0].addr[3]\
				, my_ble_id_table.cells[0].addr[2], my_ble_id_table.cells[0].addr[1], my_ble_id_table.cells[0].addr[0]);
			if(my_ble_id_table.cells[0].tx_power_level == -127){
				printf("  ?     ");
			}else{
                printf("%3d dBm ",my_ble_id_table.cells[0].tx_power_level);
            
            }
			switch (my_ble_id_table.cells[0].m_data_0_dev_type){
			    case 1:
				    printf("bat:%3d%% rec:%3d type: ArmA M1.2\n", my_ble_id_table.cells[0].m_data_1_bat_lvl, my_ble_id_table.cells[0].m_data_2_rec_flag);
			        break;
			    case 2:
				    printf("bat:%3d%% rec:%3d type: ArmA M1.3\n", my_ble_id_table.cells[0].m_data_1_bat_lvl, my_ble_id_table.cells[0].m_data_2_rec_flag);
			        break;
			    case 3:
                    printf("bat:%3d%% rec:%3d type: ArmA M1.6\n", my_ble_id_table.cells[0].m_data_1_bat_lvl, my_ble_id_table.cells[0].m_data_2_rec_flag);
                    break;
			    default:
                    printf("                 type: unknown\n");
                    break;
			};

			my_ble_id_table.count--;
		};
	}

	close(fd);

	return ret;
}
