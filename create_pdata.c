/* gcc -o create_pdata create_pdata.c */
/* supported devices:
 * mcp2515
 */
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

struct mcp251x_platform_data {
	unsigned long oscillator_frequency;
};

int main(int argc, char *argv[]) {
	void *pdata;
	char fname[256] = { 0 };
	int fd;
	uint8_t chn = 0;
	int ret;

	if (argc < 3) {
		printf("usage - %s <device> <values>\n",
		       argv[0]);
		return -EINVAL;
	}

	strcpy(fname, argv[1]);
	strcat(fname, ".pdata");
	fd = open(fname, O_CREAT | O_TRUNC | O_WRONLY,
		  S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
	if (fd < 0) {
		printf("failed to open file %s\n", fname);
		return -EINVAL;
	}

	pdata = calloc(1, sizeof(struct mcp251x_platform_data));
	if (!pdata) {
		printf("failed to allocate memory for pdata\n");
		ret = -ENOMEM;
		goto out_file;
	}

	if (!strcmp(argv[1], "mcp2515")) {
		((struct mcp251x_platform_data*) pdata)->
			oscillator_frequency = strtoul(argv[2], NULL, 0);
		ret = write(fd, &chn, sizeof(uint8_t));
		if (ret < sizeof(uint8_t)) {
			printf("failed to wite file %s\n", fname);
			ret = -EIO;
			goto out_mem;
		}
		ret = write(fd, pdata, sizeof(struct mcp251x_platform_data));
		if (ret < sizeof(struct mcp251x_platform_data)) {
			printf("failed to wite file %s\n", fname);
			ret = -EIO;
			goto out_mem;
		}
	}

out_mem:
	free(pdata);

out_file:
	close(fd);

	return ret;
}
