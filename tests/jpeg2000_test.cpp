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

#include <metrics.hpp>

using namespace pcl_compress;


int main (int argc, char const* argv[]) {
	std::string  dir_out;
	std::string  prefix_out;


	po::options_description desc("jpeg2000_test command line options");
	desc.add_options()
		("help,h",  "Help message")
		("input-file,i",     po::value<std::vector<std::string>>(), "Input files")
		("output-dir,o",     po::value<std::string>(&dir_out)->default_value("."), "Output directory (defaults to working directory)")
		("prefix,p",  po::value<std::string>(&prefix_out), "Prefix to use for output files (defaults to input file basename)")
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

    fs::path path_out(dir_out);
    if (!fs::exists(dir_out)) {
        try {
            fs::create_directories(path_out);
        } catch (std::exception& e) {
            std::cerr << "Unable to create output directory \"" << dir_out << "\"" << "\n";
            std::cerr << e.what() << "\n";
            return 1;
        }
    }

    std::vector<image_t> images;

    if (!vm.count("input-file")) {
        std::cout << "No input files given. Aborting." << "\n";
        return 0;
    }

    std::vector<std::string> input_files = vm["input-file"].as<std::vector<std::string>>();
    for (const auto& input_file : input_files) {
        fs::path p(input_file);
        if (fs::exists(p)) {
            image_t img = cv::imread(p.string(), CV_LOAD_IMAGE_GRAYSCALE);
            images.push_back(img);
        }
    }

    chunks_t stream_data = jpeg2000_compress_images(images, 35);
    for (uint32_t idx = 0; idx < stream_data.size(); ++idx) {
        chunk_ptr_t chunk = stream_data[idx];
        std::string outfile = "/tmp/out/"+std::to_string(idx++)+".j2k";
        std::ofstream out(outfile.c_str());
        out.write((const char*)chunk->data(), chunk->size());
        out.close();

        //std::ifstream in(outfile.c_str());
        //chunk_ptr_t in_chunk(new chunk_t());
        //in_chunk->length = chunk->length;
        //in_chunk->data = new uint8_t[chunk->length];
        //in.read((char*)in_chunk->data, in_chunk->length);
        //in.close();
        image_t in_img = jpeg2000_decompress_chunk(chunk);
        std::string outjpeg = "/tmp/out/"+std::to_string(idx++)+".jpg";
        cv::imwrite(outjpeg, in_img);
    }

    //if (stream_data.chunks.size() != input_files.size()) {
        //std::cerr << "Not all images compressed" << "\n";
        //return 1;
    //}
    //std::cout << "done compressing." << "\n";

    //uint64_t num_bytes = static_cast<uint64_t>(stream_data.global->length);
    //for (const auto& chunk : stream_data.chunks) {
        //num_bytes += static_cast<uint64_t>(chunk->length);
    //}
    //std::cout << "num bytes in compressed form: " << (num_bytes / 1024) << "kB.\n";


    //std::shared_ptr<stream_context> context = init_stream_decompress(stream_data.global);
    //for (uint32_t i = 0; i < input_files.size(); ++i) {
        //std::string out_path;
        //if (!vm.count("prefix")) {
            //fs::path in_path(input_files[i]);
            //out_path = (path_out / fs::basename(in_path.stem())).string() + ".jpg";
        //} else {
            //out_path = (path_out / prefix_out).string() + std::to_string(i-1) + ".jpg";
        //}
        //image_t rec = decompress_chunk(context, stream_data.chunks[i]);
        //cv::imwrite(out_path, rec);
    //}
}

