#include <iostream>
#include <fstream>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

#include <compress.hpp>
#include <decompress.hpp>
#include <zlib.hpp>

using namespace pcl_compress;


int main (int argc, char const* argv[]) {
    std::vector<std::string>  files_in;
	std::string  file_out;

	po::options_description desc("decompress_patches command line options");
	desc.add_options()
		("help,h",  "Help message")
		("input-file,i",     po::value<std::vector<std::string>>(&files_in), "Input files")
		("output-file,o",    po::value<std::string>(&file_out)->required(), "Output file")
	;

    po::positional_options_description p;
    p.add("input-file", -1);

	// Check for required options.
	po::variables_map vm;
	bool optionsException = false;
	try {
		po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
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


    //fs::path path_in(file_in);
    //fs::path path_out(file_out);

    compressed_cloud_t result;
    merged_global_data_t merged_gdata;

    for (const auto& file : files_in) {
        std::cout << "...processing \"" << file << "\"" << "\n";
        compressed_cloud_t::ptr_t cc(new compressed_cloud_t());
        deserialize(BINARY, file, *cc);

        // parse global data
        std::stringstream gcompr, gdata;
        gcompr.write((const char*)cc->global_data.data(), cc->global_data.size());
        gcompr.seekg(0);
        zlib_decompress_stream(gcompr, gdata);
        gdata.seekg(0);

        global_data_t parsed = parse_global_data(gdata);
        merged_gdata.scan_origins.push_back(parsed.scan_origin);
        merged_gdata.scan_indices.push_back(parsed.scan_index);
        merged_gdata.patch_counts.push_back(parsed.num_patches);
        merged_gdata.bbs_o.push_back(parsed.bb_o);
        merged_gdata.bbs_b.push_back(parsed.bb_b);
        merged_gdata.point_counts.insert(merged_gdata.point_counts.end(), parsed.point_counts.begin(), parsed.point_counts.end());
        merged_gdata.origins.insert(merged_gdata.origins.end(), parsed.origins.begin(), parsed.origins.end());
        merged_gdata.bboxes.insert(merged_gdata.bboxes.end(), parsed.bboxes.begin(), parsed.bboxes.end());
        merged_gdata.bases.insert(merged_gdata.bases.end(), parsed.bases.begin(), parsed.bases.end());

        result.patch_image_data.insert(result.patch_image_data.end(), cc->patch_image_data.begin(), cc->patch_image_data.end());
    }

    std::stringstream gcompr;
    zlib_compress_object(merged_gdata, gcompr);
    auto compr_length = gcompr.tellp();
    result.global_data.resize(compr_length);
    gcompr.seekg(0);
    gcompr.read((char*)result.global_data.data(), compr_length);

    serialize(BINARY, file_out, result);


    //std::cout << "decompressing" << "\n";
    //uint16_t scan_index;
    //vec3f_t scan_origin;
    //std::vector<patch_t> dec_patches = decompress_patches(cc, scan_index, scan_origin);
    //std::cout << "done" << "\n";

    //cloud_normal_t::Ptr cloud_out = from_patches(dec_patches);
    //std::cout << "Reconstructed cloud with " << cloud_out->size() << " points." << "\n";
    //pcl::io::savePCDFileBinary(path_out.string(), *cloud_out);
}
