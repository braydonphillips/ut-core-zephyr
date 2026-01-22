#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <string.h>

/* ===== Devicetree ===== */
#define SPI_BUS_NODE DT_NODELABEL(spi2)

static const struct device *spi_bus = DEVICE_DT_GET(SPI_BUS_NODE);

/* Pull CS GPIO spec directly from spi2's cs-gpios */
static const struct gpio_dt_spec cs_gpio =
	GPIO_DT_SPEC_GET_BY_IDX(SPI_BUS_NODE, cs_gpios, 0);

/* ===== FRAM opcode ===== */
#define OPC_RDID 0x9F

/* ===== SPI config (AUTO CS) =====
 * CY15B104 supports Mode 0 or Mode 3.
 * You found Mode 3 works on your wiring/timing, so lock it in.
 */
static struct spi_config spi_cfg_auto_cs = {
	.frequency = 250000,
	.operation = SPI_OP_MODE_MASTER
		   | SPI_TRANSFER_MSB
		   | SPI_WORD_SET(8)
		   | SPI_MODE_CPOL
		   | SPI_MODE_CPHA,   /* MODE 3 */
	.slave = 0,
	.cs = {
		.gpio = cs_gpio,
		.delay = 0,
	},
};

static void dump_bytes(const char *label, const uint8_t *b, size_t n)
{
	printk("%s", label);
	for (size_t i = 0; i < n; i++) {
		printk("%02X ", b[i]);
	}
	printk("\n");
}

static void reverse_bytes(uint8_t *buf, size_t n)
{
	for (size_t i = 0; i < n / 2; i++) {
		uint8_t t = buf[i];
		buf[i] = buf[n - 1 - i];
		buf[n - 1 - i] = t;
	}
}

/* Send 1-byte command, then clock out rx_len bytes */
static int spi_cmd_read(uint8_t cmd, uint8_t *rx, size_t rx_len)
{
	struct spi_buf tx_bufs[2] = {
		{ .buf = &cmd, .len = 1 },
		{ .buf = NULL, .len = rx_len },
	};
	struct spi_buf rx_bufs[2] = {
		{ .buf = NULL, .len = 1 },
		{ .buf = rx,   .len = rx_len },
	};

	struct spi_buf_set tx = { .buffers = tx_bufs, .count = 2 };
	struct spi_buf_set rxset = { .buffers = rx_bufs, .count = 2 };

	return spi_transceive(spi_bus, &spi_cfg_auto_cs, &tx, &rxset);
}

static int fram_rdid(uint8_t id[9])
{
    uint8_t tx[10] = { OPC_RDID };   /* cmd + 9 dummy */
    uint8_t rx[10] = { 0 };

    struct spi_buf txb = { .buf = tx, .len = sizeof(tx) };
    struct spi_buf rxb = { .buf = rx, .len = sizeof(rx) };
    struct spi_buf_set txset = { .buffers = &txb, .count = 1 };
    struct spi_buf_set rxset = { .buffers = &rxb, .count = 1 };

    int ret = spi_transceive(spi_bus, &spi_cfg_auto_cs, &txset, &rxset);
    if (ret) {
        return ret;
    }

    memcpy(id, &rx[1], 9);  /* skip echoed cmd byte */
    return 0;
}

int main(void)
{
	printk("Boot\n");

	if (!device_is_ready(spi_bus)) {
		printk("SPI2 not ready\n");
		return 0;
	}
	if (!gpio_is_ready_dt(&cs_gpio)) {
		printk("CS GPIO not ready\n");
		return 0;
	}

	printk("CS from DT: port=%s pin=%d dt_flags=0x%x\n",
	       cs_gpio.port->name, cs_gpio.pin, cs_gpio.dt_flags);

	/* Datasheet tPU min 450 us -> wait 1ms */
	k_busy_wait(1000);

	uint8_t id[9];
	int ret = fram_rdid(id);
	if (ret) {
		printk("RDID failed: %d\n", ret);
		while (1) { k_msleep(1000); }
	}

	dump_bytes("RDID:          ", id, 9);

	uint8_t id_rev[9];
	memcpy(id_rev, id, 9);
	reverse_bytes(id_rev, 9);
	dump_bytes("RDID(reversed): ", id_rev, 9);

	/* Expected: 7F 7F 7F 7F 7F 7F C2 2C 00 */
	const uint8_t exp[9] = { 0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0xC2,0x2C,0x00 };

	if (memcmp(id, exp, 9) == 0) {
		printk("RDID matches expected (normal order)\n");
	} else if (memcmp(id_rev, exp, 9) == 0) {
		printk("RDID matches expected (but bytes printed reversed)\n");
	} else {
		printk("RDID mismatch\n");
		printk("Expected:       7F 7F 7F 7F 7F 7F C2 2C 00\n");
	}

	while (1) {
		k_msleep(1000);
	}
}
