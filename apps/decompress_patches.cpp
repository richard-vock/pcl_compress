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

	po::options_description desc("decompress_patches command line options");
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

    compressed_cloud_t::ptr_t cc(new compressed_cloud_t());
    deserialize(BINARY, file_in, *cc);

    std::cout << "decompressing" << "\n";
    std::vector<patch_t> dec_patches = decompress_patches(cc);
    std::cout << "done" << "\n";

    cloud_xyz_t::Ptr cloud_out = from_patches(dec_patches);
    pcl::io::savePCDFileBinary(path_out.string(), *cloud_out);
}
