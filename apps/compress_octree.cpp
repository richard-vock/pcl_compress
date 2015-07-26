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
#include <decomposition.hpp>

#include <pcl/io/pcd_io.h>

using namespace pcl_compress;


int main (int argc, char const* argv[]) {
	std::string  file_in;
	std::string  file_out;
    float        px_factor;
    float        px_epsilon;
    vec2i_t      min_img_size;

	po::options_description desc("jpeg2000_test command line options");
	desc.add_options()
		("help,h",  "Help message")
		("input-file,i",     po::value<std::string>(&file_in)->required(), "Input file")
		("output-file,o",    po::value<std::string>(&file_out)->required(), "Output file")
		("pixel-factor,f",   po::value<float>(&px_factor)->default_value(10.f), "Pixel size factor")
		("pixel-epsilon,e",   po::value<float>(&px_epsilon)->default_value(1E-4), "Patch resolution epsilon")
		("min-img-x,x",   po::value<int>(&min_img_size[0])->default_value(20), "Minimum image width")
		("min-img-y,y",   po::value<int>(&min_img_size[1])->default_value(20), "Minimum image height")
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

    cloud_xyz_t::Ptr cloud_in(new cloud_xyz_t());
    pcl::io::loadPCDFile(path_in.string(), *cloud_in);

    bbox3f_t bbox;
    for (const auto& p : cloud_in->points) {
        bbox.extend(p.getVector3fMap());
    }
    decomposition_t decomp = octree_decomposition<point_xyz_t>(cloud_in, 0.03f * bbox.diagonal().norm());

    std::vector<patch_t> patches;
    for (const auto& subset : decomp) {
        patch_t patch = compute_patch(cloud_in, subset, px_factor, px_epsilon, min_img_size);
        patches.push_back(patch);
    }

    std::cout << "compressing" << "\n";
    compressed_cloud_t::ptr_t cc = compress_patches(patches, 35);
    uint32_t chars = 4 * 4 + 2*cc->origins.size() + 2*cc->bboxes.size() + cc->bases.size();
    for (const auto& chunk : cc->patch_image_data) {
        chars += chunk.size();
    }

    uint64_t bytes_in = static_cast<uint64_t>(cloud_in->size() * 12);
    uint64_t bytes_out = static_cast<uint64_t>(chars);
    uint64_t c_factor = bytes_in / bytes_out;
    double bpp = static_cast<double>(bytes_out) / (static_cast<double>(cc->num_points) / 8.0);
    std::cout << "compression factor: " << c_factor << "\n";
    std::cout << "bpp: " << bpp << "\n";

    serialize(BINARY, file_out, *cc);

    //std::ofstream out(file_out.c_str());
    //if (!out.good()) {
        //std::cerr << "Unable to open file \"" << file_out << "\" for writing." << "\n";
        //return 1;
    //}
    //out.write((const char*)&cc->num_patches, 4);
    //out.write((const char*)&cc->num_points, 4);
    //out.write((const char*)&cc->bbox_origins.min()[0], 4);
    //out.write((const char*)&cc->bbox_origins.min()[1], 4);
    //out.write((const char*)&cc->bbox_origins.min()[2], 4);
    //out.write((const char*)&cc->bbox_origins.max()[0], 4);
    //out.write((const char*)&cc->bbox_origins.max()[1], 4);
    //out.write((const char*)&cc->bbox_origins.max()[2], 4);
    //out.write((const char*)&cc->bbox_bboxes.min()[0], 4);
    //out.write((const char*)&cc->bbox_bboxes.min()[1], 4);
    //out.write((const char*)&cc->bbox_bboxes.min()[2], 4);
    //out.write((const char*)&cc->bbox_bboxes.max()[0], 4);
    //out.write((const char*)&cc->bbox_bboxes.max()[1], 4);
    //out.write((const char*)&cc->bbox_bboxes.max()[2], 4);

    //const char* data = reinterpret_cast<const char*>(cc->origins.data());
    //out.write(data, cc->origins.size()*sizeof(uint16_t));

    //data = reinterpret_cast<const char*>(cc->bboxes.data());
    //out.write(data, cc->bboxes.size()*sizeof(uint16_t));

    //data = reinterpret_cast<const char*>(cc->bases.data());
    //out.write(data, cc->bases.size()*sizeof(uint8_t));

    //for (const auto& chunk : cc->patch_image_data) {
        //out.write((const char*)&chunk->length, sizeof(int));
        //out.write((const char*)&chunk->data, sizeof(uint8_t)*chunk->length);
    //}
    //out.close();
}
