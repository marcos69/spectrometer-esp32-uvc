/* Gray-decode wrapper over vendored libjpeg (jpeg-9a port, Apache-2.0).
 * JCS_GRAYSCALE on a 3-component JPEG yields the Y plane exactly.
 */
#include "jpegd.h"
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "jpeglib.h"

struct my_error_mgr {
    struct jpeg_error_mgr pub;
    jmp_buf setjmp_buffer;
};

static void my_error_exit(j_common_ptr cinfo)
{
    struct my_error_mgr *m = (struct my_error_mgr *)cinfo->err;
    (*cinfo->err->output_message)(cinfo);
    longjmp(m->setjmp_buffer, 1);
}

int jpegd_scan_info(const uint8_t *src, size_t len, uint32_t *w, uint32_t *h)
{
    struct jpeg_decompress_struct cinfo;
    struct my_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr.pub);
    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, (unsigned char *)src, (unsigned long)len);
    jpeg_read_header(&cinfo, TRUE);
    *w = (uint32_t)cinfo.image_width;
    *h = (uint32_t)cinfo.image_height;
    jpeg_destroy_decompress(&cinfo);
    return 0;
}

int jpegd_decode_gray(const uint8_t *src, size_t len, uint8_t *out,
                      uint32_t w, uint32_t h)
{
    (void)w;
    (void)h;
    struct jpeg_decompress_struct cinfo;
    struct my_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = my_error_exit;
    if (setjmp(jerr.setjmp_buffer)) {
        jpeg_destroy_decompress(&cinfo);
        return -1;
    }
    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, (unsigned char *)src, (unsigned long)len);
    jpeg_read_header(&cinfo, TRUE);
    cinfo.out_color_space = JCS_GRAYSCALE;
    jpeg_start_decompress(&cinfo);
    JSAMPARRAY row = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo,
                                                JPOOL_IMAGE, cinfo.output_width, 1);
    uint8_t *dst = out;
    while (cinfo.output_scanline < cinfo.output_height) {
        jpeg_read_scanlines(&cinfo, row, 1);
        memcpy(dst, row[0], (size_t)cinfo.output_width);
        dst += cinfo.output_width;
    }
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    return 0;
}

int jpegd_decode_gray_auto(const uint8_t *src, size_t len, uint8_t *out,
                           size_t out_cap, uint32_t *out_w, uint32_t *out_h)
{
    struct jpeg_decompress_struct cinfo;
    struct my_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = my_error_exit;
    if (setjmp(jerr.setjmp_buffer)) {
        jpeg_destroy_decompress(&cinfo);
        return -1;
    }
    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, (unsigned char *)src, (unsigned long)len);

    /* 1/2 scale for big frames: 1/4 the DCT/IDCT work. The spectrometer
     * consumes one row, so decoded resolution can be halved safely.
     * image_width/height are only valid AFTER jpeg_read_header — reading
     * them earlier silently disabled this path (always full-res decode). */
    jpeg_read_header(&cinfo, TRUE);
    const unsigned long wanted = (unsigned long)cinfo.image_width *
                                 (unsigned long)cinfo.image_height;
    if (wanted >= 400000UL) {
        cinfo.scale_num = 1;
        cinfo.scale_denom = 2;
    }
    cinfo.out_color_space = JCS_GRAYSCALE;
    jpeg_start_decompress(&cinfo);

    const size_t ow = cinfo.output_width;
    const size_t oh = cinfo.output_height;
    if (ow * oh > out_cap) {
        jpeg_destroy_decompress(&cinfo);
        return -1;
    }
    JSAMPARRAY row = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo,
                                                JPOOL_IMAGE, cinfo.output_width, 1);
    uint8_t *dst = out;
    while (cinfo.output_scanline < cinfo.output_height) {
        jpeg_read_scanlines(&cinfo, row, 1);
        memcpy(dst, row[0], ow);
        dst += ow;
    }
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    if (out_w) *out_w = (uint32_t)ow;
    if (out_h) *out_h = (uint32_t)oh;
    return 0;
}
