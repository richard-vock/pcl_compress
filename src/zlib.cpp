#include <zlib.hpp>

#ifndef ZLIB_CHUNK_SIZE
#define ZLIB_CHUNK_SIZE 262144
#endif

extern "C" {
#include <zlib.h>
}

namespace pcl_compress {

void zlib_compress_stream(std::istream& input_buffer, std::ostream& output_buffer) {
    int ret, flush;
    unsigned have;
    z_stream strm;
    unsigned char in[ZLIB_CHUNK_SIZE];
    unsigned char out[ZLIB_CHUNK_SIZE];

    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    int level = Z_DEFAULT_COMPRESSION;
    ret = deflateInit(&strm, level);
    if (ret != Z_OK) {
        throw std::runtime_error("zlib_compress_stream: Failed initializing stream");
    }

    do {
        input_buffer.read((char*)in, ZLIB_CHUNK_SIZE);
        strm.avail_in = input_buffer.gcount();
        if (!input_buffer && !input_buffer.eof()) {
            (void)deflateEnd(&strm);
            throw std::runtime_error("zlib_compress_stream: Unexpected buffer underflow");
        }
        flush = input_buffer.eof() ? Z_FINISH : Z_NO_FLUSH;
        strm.next_in = in;

        do {
            strm.avail_out = ZLIB_CHUNK_SIZE;
            strm.next_out = out;
            ret = deflate(&strm, flush);
            assert(ret != Z_STREAM_ERROR);
            have = ZLIB_CHUNK_SIZE - strm.avail_out;
            output_buffer.write((const char*)out, have);
        } while (strm.avail_out == 0);
        assert(strm.avail_in == 0);

    } while (flush != Z_FINISH);
    assert(ret == Z_STREAM_END);

    output_buffer.flush();

    (void)deflateEnd(&strm);
}

int zlib_decompress_stream(std::istream& input_buffer, std::ostream& output_buffer) {
    int ret;
    unsigned have;
    z_stream strm;

    std::unique_ptr<unsigned char[]> in(new unsigned char[ZLIB_CHUNK_SIZE]);
    std::unique_ptr<unsigned char[]> out(new unsigned char[ZLIB_CHUNK_SIZE]);

    /* allocate inflate state */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
    ret = inflateInit(&strm);
    if (ret != Z_OK) {
        return 1;
    }

    /* decompress until deflate stream ends or end of file */
    do {
        input_buffer.read((char*)in.get(), ZLIB_CHUNK_SIZE);
        strm.avail_in = input_buffer.gcount();
        if (!input_buffer && !input_buffer.eof()) {
            (void)inflateEnd(&strm);
            return 1;
        }

        if (strm.avail_in == 0)
            break;
        strm.next_in = in.get();

        /* run inflate() on input until output buffer not full */
        do {
            strm.avail_out = ZLIB_CHUNK_SIZE;
            strm.next_out = out.get();
            ret = inflate(&strm, Z_NO_FLUSH);
            assert(ret != Z_STREAM_ERROR);  /* state not clobbered */
            switch (ret) {
            case Z_NEED_DICT:
                return 1;
            case Z_DATA_ERROR:
                return 1;
            case Z_MEM_ERROR:
                (void)inflateEnd(&strm);
                return 1;
            }
            have = ZLIB_CHUNK_SIZE - strm.avail_out;
            output_buffer.write((const char*)out.get(), have);
        } while (strm.avail_out == 0);

        /* done when inflate() says it's done */
    } while (ret != Z_STREAM_END);

    /* clean up and return */
    (void)inflateEnd(&strm);
    if (ret != Z_STREAM_END) {
        return 1;
    }

    return 0;
}

int zlib_decompress_stream_heap(std::istream& input_buffer, std::ostream& output_buffer) {
    int ret;
    unsigned have;
    z_stream strm;

    std::unique_ptr<unsigned char[]> in(new unsigned char[ZLIB_CHUNK_SIZE]);
    std::unique_ptr<unsigned char[]> out(new unsigned char[ZLIB_CHUNK_SIZE]);

    /* allocate inflate state */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
    ret = inflateInit(&strm);
    if (ret != Z_OK) {
        return 1;
    }

    /* decompress until deflate stream ends or end of file */
    do {
        input_buffer.read((char*)in.get(), ZLIB_CHUNK_SIZE);
        strm.avail_in = input_buffer.gcount();
        if (!input_buffer && !input_buffer.eof()) {
            (void)inflateEnd(&strm);
            return 1;
        }

        if (strm.avail_in == 0)
            break;
        strm.next_in = in.get();

        /* run inflate() on input until output buffer not full */
        do {
            strm.avail_out = ZLIB_CHUNK_SIZE;
            strm.next_out = out.get();
            ret = inflate(&strm, Z_NO_FLUSH);
            assert(ret != Z_STREAM_ERROR);  /* state not clobbered */
            switch (ret) {
            case Z_NEED_DICT:
                return 1;
            case Z_DATA_ERROR:
                return 1;
            case Z_MEM_ERROR:
                (void)inflateEnd(&strm);
                return 1;
            }
            have = ZLIB_CHUNK_SIZE - strm.avail_out;
            output_buffer.write((const char*)out.get(), have);
        } while (strm.avail_out == 0);

        /* done when inflate() says it's done */
    } while (ret != Z_STREAM_END);

    /* clean up and return */
    (void)inflateEnd(&strm);
    if (ret != Z_STREAM_END) {
        return 1;
    }

    return 0;
}

//void zlib_decompress_stream(std::istream& input_buffer, std::ostream& output_buffer) {
    //int ret;
    //unsigned have;
    //z_stream strm;
    //unsigned char in[ZLIB_CHUNK_SIZE];
    //unsigned char out[ZLIB_CHUNK_SIZE];

    //strm.zalloc = Z_NULL;
    //strm.zfree = Z_NULL;
    //strm.opaque = Z_NULL;
    //strm.avail_in = 0;
    //strm.next_in = Z_NULL;
    //ret = inflateInit(&strm);
    //if (ret != Z_OK) {
        //throw std::runtime_error("zlib_decompress_stream: Failed initializing stream");
    //}

    //do {
        //input_buffer.read((char*)in, ZLIB_CHUNK_SIZE);
        //strm.avail_in = 0;
        //if (!input_buffer && !input_buffer.eof()) {
            //(void)inflateEnd(&strm);
            //throw std::runtime_error("zlib_decompress_stream: Unexpected buffer underflow");
        //}

        //if (strm.avail_in == 0)
            //break;
        //strm.next_in = in;

        //do {
            //strm.avail_out = ZLIB_CHUNK_SIZE;
            //strm.next_out = out;
            //ret = inflate(&strm, Z_NO_FLUSH);
            //assert(ret != Z_STREAM_ERROR);
            //switch (ret) {
            //case Z_NEED_DICT:
                //throw std::runtime_error("zlib_decompress_stream: Data error.");
            //case Z_DATA_ERROR:
                //throw std::runtime_error("zlib_decompress_stream: Data error.");
            //case Z_MEM_ERROR:
                //(void)inflateEnd(&strm);
                //throw std::runtime_error("zlib_decompress_stream: Memory error.");
            //}
            //have = ZLIB_CHUNK_SIZE - strm.avail_out;
            //output_buffer.write((const char*)out, have);
        //} while (strm.avail_out == 0);
    //} while (ret != Z_STREAM_END);

    //(void)inflateEnd(&strm);
    //if (ret != Z_STREAM_END) {
        //throw std::runtime_error("zlib_decompress_stream: Data error.");
    //}
//}

chunk_ptr_t zlib_compress_chunk(chunk_const_ptr_t input) {
    std::stringstream input_buffer, output_buffer;
    input_buffer.write((const char*)input->data(), input->size());
    input_buffer.seekg(0);

    zlib_compress_stream(input_buffer, output_buffer);
    auto length = output_buffer.tellp();
    chunk_ptr_t output(new chunk_t(length));
    output_buffer.seekg(0);
    output_buffer.read((char*)output->data(), length);
    return output;
}

chunk_ptr_t zlib_decompress_chunk(chunk_const_ptr_t input) {
    std::stringstream input_buffer, output_buffer;
    input_buffer.write((const char*)input->data(), input->size());
    input_buffer.seekg(0);

    zlib_decompress_stream(input_buffer, output_buffer);
    auto length = output_buffer.tellp();
    chunk_ptr_t output(new chunk_t(length));
    output_buffer.seekg(0);
    output_buffer.read((char*)output->data(), length);
    return output;
}


} // pcl_compress
