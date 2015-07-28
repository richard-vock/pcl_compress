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


	po::options_description desc("jbig2_test command line options");
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
            auto s = img.size();
            for (uint32_t row = 0; row < static_cast<uint32_t>(s.height); ++row) {
                uint8_t* row_ptr = img.ptr(row);
                for (uint32_t col = 0; col < static_cast<uint32_t>(s.width); ++col) {
                    row_ptr[col] = row_ptr[col] > 188 ? 255 : 0;
                }
            }

            images.push_back(img);
        }
    }

    chunks_t stream_data = jbig2_compress_images(images);

    if (stream_data.size() != input_files.size()) {
        std::cerr << "Not all images compressed" << "\n";
        return 1;
    }
    std::cout << "done compressing." << "\n";

    uint64_t num_bytes = 0;
    for (const auto& chunk : stream_data) {
        num_bytes += static_cast<uint64_t>(chunk->size());
    }
    std::cout << "num bytes in compressed form: " << (num_bytes / 1024) << "kB.\n";


    //std::shared_ptr<stream_context> context = init_stream_decompress(stream_data.global);
    std::vector<image_t> reconstructed;
    for (uint32_t i = 0; i < input_files.size(); ++i) {
        std::string out_path;
        if (!vm.count("prefix")) {
            fs::path in_path(input_files[i]);
            out_path = (path_out / fs::basename(in_path.stem())).string() + ".jpg";
        } else {
            out_path = (path_out / prefix_out).string() + std::to_string(i-1) + ".jpg";
        }
        image_t rec = jbig2_decompress_chunk(stream_data[i]);
        reconstructed.push_back(rec);
        cv::imwrite(out_path, rec);
    }
    std::cout << "rmse: " << compute_rmse(images, reconstructed) << "\n";
}

