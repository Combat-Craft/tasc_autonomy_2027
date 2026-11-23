#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;

// Metadata for a captured image
struct FrameMetadata
{
    std::string frame;
    std::string gps_latitude;
    std::string gps_longitude;
    std::string camera_heading;
    std::string camera_cardinal;
};

// Expand home directory
fs::path expand_home(const std::string & path) {
    if (path == "~" || path.rfind("~/", 0) == 0) {
        const char * home = std::getenv("HOME");
        if (!home) {
            throw std::runtime_error("HOME is not set");
        }

        return fs::path(home) / path.substr(path == "~" ? 1 : 2);
    }

    return fs::path(path);
}

// Load panorama images
std::vector<cv::Mat> load_images(const fs::path & folder, std::vector<std::string> & filenames) {
    std::vector<fs::path> image_paths;
    std::vector<cv::Mat> frames;

    // Find frame_*.jpg files
    for (const auto & entry : fs::directory_iterator(folder)) {
        if (!entry.is_regular_file()) {
            continue;
        }

        const std::string filename = entry.path().filename().string();

        if (filename.rfind("frame_", 0) == 0 && entry.path().extension() == ".jpg") {
            image_paths.push_back(entry.path());
        }
    }

    // Sort images by filename
    std::sort(image_paths.begin(), image_paths.end());

    if (image_paths.size() < 2) {
        std::cerr
            << "Need at least 2 images to stitch. Found "
            << image_paths.size() << ".\n";

        return frames;
    }

    // Load each image
    for (const auto & image_path : image_paths) {
        cv::Mat frame = cv::imread(image_path.string(), cv::IMREAD_COLOR);

        if (frame.empty()) {
            std::cerr
                << "Warning: Could not read "
                << image_path << "\n";

            continue;
        }

        frames.push_back(frame);
        filenames.push_back(image_path.filename().string());
    }

    std::cout
        << "Loaded " << frames.size()
        << " valid images from " << folder << "\n";

    return frames;
}

// Parse CSV line (used for getting individual metadata values)
std::vector<std::string> parse_csv_line(const std::string & line) {
    std::vector<std::string> values;
    std::string current;
    bool inside_quotes = false;

    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];

        if (c == '"') {
            // Handle escaped double quotes
            if (inside_quotes && i + 1 < line.size() && line[i + 1] == '"') {
                current += '"';
                ++i;
            }
            else {
                inside_quotes = !inside_quotes;
            }
        }
        else if (c == ',' && !inside_quotes) {
            values.push_back(current);
            current.clear();
        }
        else if (c != '\r') {
            current += c;
        }
    }

    values.push_back(current);
    return values;
}

// Load metadata.csv
std::vector<FrameMetadata> load_metadata(const fs::path & folder) {
    std::vector<FrameMetadata> metadata;

    const fs::path metadata_path = folder / "metadata.csv";

    std::ifstream file(metadata_path);

    if (!file) {
        std::cout
            << "No metadata.csv found. "
            << "Stitching without metadata overlay.\n";

        return metadata;
    }

    std::string line;

    // Read CSV header
    if (!std::getline(file, line)) {
        return metadata;
    }

    const auto headers = parse_csv_line(line);

    std::unordered_map<std::string, std::size_t> columns;

    for (std::size_t i = 0; i < headers.size(); ++i) {
        columns[headers[i]] = i;
    }

    // Retrieve a field by column name
    auto get_field = [&columns](
        const std::vector<std::string> & values,
        const std::string & name) -> std::string
    {
        const auto it = columns.find(name);

        if (it == columns.end() ||
            it->second >= values.size())
        {
            return "";
        }

        return values[it->second];
    };

    // Read each metadata row
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }

        const auto values = parse_csv_line(line);

        FrameMetadata item;

        item.frame = get_field(values, "frame");

        item.gps_latitude = get_field(values, "gps_latitude");

        item.gps_longitude = get_field(values, "gps_longitude");

        item.camera_heading = get_field(values, "camera_heading");

        item.camera_cardinal = get_field(values, "camera_cardinal");

        metadata.push_back(item);
    }

    std::cout
        << "Loaded metadata for "
        << metadata.size() << " frames\n";

    return metadata;
}

// Draw text with black shadow and white foreground
void draw_text(
    cv::Mat & image,
    const std::string & text,
    const cv::Point & position,
    double scale)
{
    const int font = cv::FONT_HERSHEY_SIMPLEX;

    cv::putText(
        image, text, position,
        font, scale,
        cv::Scalar(0, 0, 0),
        3, cv::LINE_AA);

    cv::putText(
        image, text, position,
        font, scale,
        cv::Scalar(255, 255, 255),
        1, cv::LINE_AA);
}

// Draw GPS coordinates
void draw_gps_label(
    cv::Mat & pano,
    const std::vector<FrameMetadata> & metadata)
{
    if (metadata.empty())
    {
        return;
    }

    const auto & first = metadata.front();

    if (first.gps_latitude.empty() || first.gps_longitude.empty()) {
        return;
    }

    const std::string gps_text =
        "GPS: " +
        first.gps_latitude + ", " +
        first.gps_longitude;

    draw_text(pano, gps_text, cv::Point(20, 30), 0.6);
}

// Format camera heading
std::string format_heading(const std::string & heading)
{
    if (heading.empty()) {
        return "N/A";
    }

    try {
        std::size_t parsed = 0;
        const double value = std::stod(
            heading, &parsed);

        if (parsed != heading.size() || !std::isfinite(value)) {
            return heading;
        }

        std::ostringstream output;

        output << std::fixed
               << std::setprecision(0)
               << value;

        return output.str();
    }
    catch (const std::exception &) {
        return heading;
    }
}

// Draw cardinal direction and heading labels
void draw_cardinal_labels(
    cv::Mat & pano,
    const std::vector<FrameMetadata> & metadata,
    const std::vector<std::string> & filenames)
{
    if (metadata.empty()) {
        return;
    }

    // Match metadata to captured image filenames
    std::unordered_map<std::string, FrameMetadata>
        metadata_by_frame;

    for (const auto & item : metadata) {
        metadata_by_frame[item.frame] = item;
    }

    const int width = pano.cols;
    const std::size_t count = filenames.size();

    for (std::size_t i = 0; i < count; ++i) {
        const auto it = metadata_by_frame.find(
            filenames[i]);

        if (it == metadata_by_frame.end())
        {
            continue;
        }

        const auto & item = it->second;

        // Skip labels until heading data is available
        if (item.camera_cardinal.empty() &&
            item.camera_heading.empty())
        {
            continue;
        }

        // Approximate horizontal position
        int x = width / 2;

        if (count > 1)
        {
            const double fraction =
                static_cast<double>(i) / (count - 1);

            x = static_cast<int>(
                fraction * (width - 40)) + 20;
        }

        const std::string cardinal =
            item.camera_cardinal.empty()
                ? "?"
                : item.camera_cardinal;

        const std::string heading =
            format_heading(item.camera_heading);

        // Draw cardinal direction
        draw_text(
            pano, cardinal,
            cv::Point(x - 15, 55), 0.6);

        // Draw numeric heading
        draw_text(
            pano, heading,
            cv::Point(x - 20, 78), 0.45);

        // Draw tick mark below label
        cv::line(
            pano,
            cv::Point(x, 90),
            cv::Point(x, 110),
            cv::Scalar(255, 255, 255),
            1);
    }
}

// -------------------------------------------------
// Panorama stitching
// -------------------------------------------------

bool stitch_images(const std::string & folder_path)
{
    const fs::path folder =
        expand_home(folder_path);

    // Validate input folder
    if (!fs::exists(folder) ||
        !fs::is_directory(folder))
    {
        std::cerr
            << "Invalid folder: " << folder << "\n";

        return false;
    }

    // Load images
    std::vector<std::string> filenames;

    std::vector<cv::Mat> frames =
        load_images(folder, filenames);

    if (frames.size() < 2)
    {
        std::cerr
            << "Not enough valid images to stitch. "
            << "Found " << frames.size() << ".\n";

        return false;
    }

    // Load metadata if available
    const auto metadata = load_metadata(folder);

    std::cout
        << "Stitching " << frames.size()
        << " images from: " << folder << "\n";

    // Create OpenCV panorama stitcher
    cv::Ptr<cv::Stitcher> stitcher =
        cv::Stitcher::create(cv::Stitcher::PANORAMA);

    cv::Mat pano;

    const cv::Stitcher::Status status =
        stitcher->stitch(frames, pano);

    std::cout
        << "OpenCV stitcher status: "
        << static_cast<int>(status) << "\n";

    if (status != cv::Stitcher::OK)
    {
        std::cerr
            << "Panorama stitching failed with status "
            << static_cast<int>(status) << "\n";

        return false;
    }

    if (pano.empty())
    {
        std::cerr
            << "Stitcher returned an empty panorama.\n";

        return false;
    }

    // Draw metadata labels, if available
    draw_gps_label(pano, metadata);

    draw_cardinal_labels(
        pano, metadata, filenames);

    // Save final panorama
    const fs::path output_path =
        folder / "panorama.jpg";

    if (!cv::imwrite(output_path.string(), pano))
    {
        std::cerr
            << "Failed to save panorama: "
            << output_path << "\n";

        return false;
    }

    std::cout
        << "Panorama saved: "
        << output_path << "\n";

    return true;
}

int main(int argc, char * argv[])
{
    std::string folder_path;

    // Allow folder path as command-line argument
    if (argc == 2) {
        folder_path = argv[1];
    }
    else if (argc == 1) {
        std::cout
            << "Enter folder path containing "
            << "frame_*.jpg images: ";

        std::getline(std::cin, folder_path);
    }
    else {
        std::cerr
            << "Usage: " << argv[0]
            << " [sweep_folder]\n";

        return 1;
    }

    if (folder_path.empty()) {
        std::cerr << "No folder path entered.\n";
        return 1;
    }

    try {
        return stitch_images(folder_path) ? 0 : 1;
    }
    catch (const cv::Exception & e) {
        std::cerr
            << "OpenCV error: " << e.what() << "\n";
    }
    catch (const std::exception & e) {
        std::cerr
            << "Error: " << e.what() << "\n";
    }

    return 1;
}
