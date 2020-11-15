// compile it with the following command: gcc zstd_dict.cpp -o zstd_dict -I ../../../b/zstd/include/ -lzstd -L ../../../b/zstd/lib/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include <zdict.h>

using namespace std;

void usage(char* name, char* message) {
    if (message) {
        printf("%s\n\n", message);
    }
    printf("Usage:\n");;
    printf("  %s -i <input file> -s <size> -o <output file>\n", name);
    printf("    -i <input file>  : input file containing training samples\n");
    printf("    -s <size>        : size of the sample\n");
    printf("    -o <output file> : ouput file to store generated dictionary\n");

    exit(1);
}

int main(int argc, char** argv) {
    char infile_name[128] = {0};
    char outfile_name[128] = {0};
    size_t size;

    FILE* infile = {0};
    FILE* outfile = {0};
    size_t infile_size = 0;
    size_t outfile_size = 0;

    void* in = {0};
    void* out = {0};

    int opt;
    while ((opt = getopt(argc, argv, "i:s:o:h?")) != -1) {
        switch (opt) {
        case 'i':
            strcpy(infile_name, optarg);
            continue;
        case 's':
            size = atoi(optarg);
            continue;
        case 'o':
            strcpy(outfile_name, optarg);
            continue;
        case '?':
        case 'h':
            usage(argv[0], NULL);
            break;
        default :
            usage(argv[0], (char*)"Invalid parameter");
            break;
        }
    }

    if (*infile_name == 0 || size == 0 || *outfile_name == 0) usage(argv[0], (char*)"Incorrect parameter");
        
    size_t dict_size = 102400;
    void* dict = malloc(dict_size);
    if (dict == NULL) printf("Cannot allocate memory for dictionary\n");

    infile = fopen(infile_name, "rb");
    if (infile) {
        fseek(infile, 0, SEEK_END);
        infile_size = ftell(infile);
        fseek(infile, 0, SEEK_SET);
        if (infile_size % size == 0) {
            int num_elems = infile_size / size;
            printf("Reading %lu bytes from %s, %u total samples\n", infile_size, infile_name, num_elems);
            in = malloc(infile_size);
            if (in) {
                if (fread(in, 1, infile_size, infile) == infile_size) {
                    size_t* sizes = (size_t*)malloc(sizeof(size_t) * num_elems);
                    if (sizes) {
                        printf("Generating sizes array\n");
                        for (int i = 0; i < num_elems; i++) sizes[i] = size;
                        printf("Generating dictionary\n");
                        outfile_size = ZDICT_trainFromBuffer(dict, dict_size, in, sizes, infile_size / size);
                        printf("Dictionary size is %lu\n", outfile_size);
                        outfile = fopen(outfile_name, "wb");
                        if (outfile) {
                            out_bytes = fwrite(dict, 1, outfile_size, outfile)
                            if (out_bytes == outfile_size) {
                                printf("Wrote %lu bytes of dictionary into %s\n", outfile_size, outfile_name);
                                fclose(outfile);
                            } else printf("ERROR: Cannot write dictionary: %lu/%lu\n", out_bytes, outfile_size);
                        } else printf("ERROR: Cannot open output file %s\n", outfile_name);
                        free(sizes);
                    } else printf("ERROR: Cannot allocate memory for sizes\n");
                } else printf("ERROR: Cannot read data from input file %s\n", infile_name);
                free(in);
            } else printf("ERROR: Cannot allocate memory\n");
        } else printf("ERROR: Input file %s contains invalid data\n", infile_name);
        fclose(infile);
    } else printf("ERROR: Cannot open input file %s\n", infile_name);

    free(dict);
    
    return 0;
}