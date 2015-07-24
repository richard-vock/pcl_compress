#include <iostream>
#include <fstream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <sys/fcntl.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

#include <compress.hpp>
#include <decompress.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>

using namespace pcl_compress;


int main (int argc, char const* argv[]) {
	std::string  file_in;
	std::string  file_out;

	po::options_description desc("jpeg2000_test command line options");
	desc.add_options()
		("help,h",  "Help message")
		("input-file,i",     po::value<std::string>(&file_in)->required(), "Input file")
		("output-file,o",    po::value<std::string>(&file_out)->required(), "Output file")
	;

	// Check for required options.
	po::variables_map vm;
	bool optionsException = false;
	try {
		po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
		//po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);
	} catch (std::exception& e) {
		if (!vm.count("help")) {
			std::cout << e.what() << "\n";
		}
		optionsException = true;
	}
	if (optionsException || vm.count("help")) {
		std::cout << desc << "\n";
		return optionsException ? 1 : 0;
	}

    fs::path path_in(file_in);
    fs::path path_out(file_out);

    if (!fs::exists(file_in)) {
        std::cerr << "Input file does not exist" << "\n";
        return 1;
    }

    std::ifstream in(file_in.c_str());
    if (!in.good()) {
        std::cerr << "Unable to open file \"" << file_in << "\" for reading." << "\n";
        return 1;
    }
    compressed_cloud_t::ptr_t cc(new compressed_cloud_t());
    in.read((char*)&cc->num_patches, 4);
    in.read((char*)&cc->num_points, 4);
    in.read((char*)&cc->bbox_origins.min()[0], 4);
    in.read((char*)&cc->bbox_origins.min()[1], 4);
    in.read((char*)&cc->bbox_origins.min()[2], 4);
    in.read((char*)&cc->bbox_origins.max()[0], 4);
    in.read((char*)&cc->bbox_origins.max()[1], 4);
    in.read((char*)&cc->bbox_origins.max()[2], 4);
    in.read((char*)&cc->bbox_bboxes.min()[0], 4);
    in.read((char*)&cc->bbox_bboxes.min()[1], 4);
    in.read((char*)&cc->bbox_bboxes.min()[2], 4);
    in.read((char*)&cc->bbox_bboxes.max()[0], 4);
    in.read((char*)&cc->bbox_bboxes.max()[1], 4);
    in.read((char*)&cc->bbox_bboxes.max()[2], 4);

    char* data = new char[3*sizeof(uint16_t)*cc->num_patches];
    in.read(data, 3*sizeof(uint16_t)*cc->num_patches);
    uint16_t* cast_data = reinterpret_cast<uint16_t*>(data);
    cc->origins = std::vector<uint16_t>(cast_data, cast_data+(3*cc->num_patches));
    delete [] data;

    data = new char[6*sizeof(uint16_t)*cc->num_patches];
    in.read(data, 6*sizeof(uint16_t)*cc->num_patches);
    cast_data = reinterpret_cast<uint16_t*>(data);
    cc->bboxes = std::vector<uint16_t>(cast_data, cast_data+(6*cc->num_patches));
    delete [] data;

    data = new char[9*sizeof(uint8_t)*cc->num_patches];
    in.read(data, 9*sizeof(uint8_t)*cc->num_patches);
    cast_data = reinterpret_cast<uint16_t*>(data);
    cc->bases = std::vector<uint8_t>(cast_data, cast_data+(9*cc->num_patches));
    delete [] data;

    for (uint32_t i = 0; i < 2*static_cast<uint32_t>(cc->num_patches); ++i) {
        chunk_ptr_t chunk(new chunk_t());
        in.read((char*)&chunk->length, sizeof(int));
        chunk->data = (uint8_t*)malloc(chunk->length * sizeof(uint8_t));
        in.read((char*)chunk->data, chunk->length * sizeof(uint8_t));
        cc->patch_image_data.push_back(chunk);
    }
    in.close();

    std::cout << "decompressing" << "\n";
    std::vector<patch_t> dec_patches = decompress_patches(cc);
    std::cout << "done" << "\n";

    cloud_xyz_t::Ptr cloud_out = from_patches(dec_patches);
    pcl::io::savePCDFileBinary(path_out.string(), *cloud_out);
}
