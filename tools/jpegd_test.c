/* Host test: jpegd wrapper vs PIL. Usage: jpegd_test <jpg> <out.raw> */
#include <stdio.h>
#include <stdlib.h>
#include "jpegd.h"

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: %s <jpg> <out.raw>\n", argv[0]); return 2; }
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 2; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *src = malloc(sz);
    if (fread(src, 1, sz, f) != (size_t)sz) { fprintf(stderr, "read fail\n"); return 2; }
    fclose(f);
    uint32_t w, h;
    if (jpegd_scan_info(src, (size_t)sz, &w, &h)) { fprintf(stderr, "scan fail\n"); return 1; }
    printf("image %ux%u\n", w, h);
    uint8_t *out = malloc((size_t)w * h);
    if (jpegd_decode_gray(src, (size_t)sz, out, w, h)) { fprintf(stderr, "decode fail\n"); return 1; }
    f = fopen(argv[2], "wb");
    fwrite(out, 1, (size_t)w * h, f);
    fclose(f);
    printf("wrote %s\n", argv[2]);
    free(out); free(src);
    return 0;
}
