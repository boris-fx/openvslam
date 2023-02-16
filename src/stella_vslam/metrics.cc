#include "metrics.h"

#include <vector>
#include <sstream>
#include <fstream>

namespace stella_vslam_bfx {

std::optional<double> video_metadata::ground_truth_pixel_aspect_ratio() const
{
    if (groundTruthFilmBackWidthMM && groundTruthFilmBackHeightMM) {
        double film_back_aspect_ratio = groundTruthFilmBackWidthMM.value() / groundTruthFilmBackHeightMM.value();
        double image_aspect_ratio = double(video_width) / double(video_height);
        return film_back_aspect_ratio / image_aspect_ratio;

    } 
    return {};
}

std::optional<double> video_metadata::ground_truth_focal_length_x_pixels() const {
    if (groundTruthFocalLengthXPixels)
        return groundTruthFocalLengthXPixels.value();
    if (groundTruthFilmBackWidthMM && groundTruthFocalLengthMM)
        return groundTruthFocalLengthMM.value() * video_width / groundTruthFilmBackWidthMM.value();
    return {};
}

std::optional<double> video_metadata::calculated_focal_length_mm(double calculated_focal_length_x_pixels) const
{
    if (groundTruthFilmBackWidthMM)
        return calculated_focal_length_x_pixels * groundTruthFilmBackWidthMM.value() / video_width;
    return {};
}

//////////////////////////////////////////////////////////////

double timings::total_time_sec() const {
    return  forward_mapping + backward_mapping + loop_closing + optimisation + tracking;
}

//////////////////////////////////////////////////////////////

const double metrics::par_percent_error_trigger = 20.0;
const double metrics::focal_percent_error_trigger = 50.0;

metrics* metrics::get_instance() {
    if (!instance)
        instance = new metrics();
    return instance;
}

void metrics::clear() {
    if (instance) {
        delete instance;
        instance = nullptr;
    }
}

int metrics::total_frames() const
{
    return solved_frame_count + unsolved_frame_count;
}

double percent_difference(double a, double b)
{
    if (a < b)
        return (100.0 * (b - a) / a);
    else
        return (100.0 * (a - b) / b);
}

bool within_percent(double a, double b, double percent) {
    return percent_difference(a, b) < percent;
}

bool within_percent(std::optional<double> a, double b, double percent) {
    if (!a.has_value())
        return true;
    return within_percent(a.value(), b, percent);
}

bool within_percent(double a, std::optional<double> b, double percent) {
    return within_percent(b, a, percent);
}

std::set<std::pair<std::string, tracking_problem_level>> metrics::problems() const
{
    std::set<std::pair<std::string, tracking_problem_level>> problem_set;
    if (unsolved_frame_count!=0)
        if (solved_frame_count==0)
            problem_set.insert({"No tracked frames", tracking_problem_fatal});
        else
            problem_set.insert({std::to_string(unsolved_frame_count)+" untracked frames", tracking_problem_warning});
    if (!within_percent(input_video_metadata.ground_truth_focal_length_x_pixels(), calculated_focal_length_x_pixels, focal_percent_error_trigger))
        problem_set.insert({"Wrong focal length", tracking_problem_warning});
    if (!within_percent(input_video_metadata.pixel_aspect_ratio_used, input_video_metadata.ground_truth_pixel_aspect_ratio(), par_percent_error_trigger))
        problem_set.insert({"Wrong par", tracking_problem_warning});
    return problem_set;
}

std::optional<tracking_problem_level> metrics::max_problem_level() const
{
    bool have_warning(false), have_fatal(false);
    auto probs = problems();
    for (auto const& problem : probs) {
        if (problem.second == tracking_problem_warning)
            have_warning = true;
        if (problem.second == tracking_problem_fatal)
            have_fatal = true;
    }
    if (have_fatal)
        return tracking_problem_fatal;
    if (have_warning)
        return tracking_problem_warning;
    return {};
}

template<typename T>
std::string to_string(std::set<T> const& s)
{
    std::string str; 
    int n(s.size()-1), i(0);
    for (auto const& item : s) {
        str += std::to_string(item);
        if (i != n)
            str += ", ";
        ++i;
    }
    return str;
}

void metrics::save_html_report(std::string_view const& filename, std::string thumbnail_file) const {
    std::ofstream myfile;
    myfile.open(filename.data());

    std::stringstream html;
    html << "<!DOCTYPE html><html>\n";
    html << "<head>\n";
    html << "  <meta charset = 'utf-8' />\n";
    html << "  <title>" << input_video_metadata.name << "</title>\n";
    html << "  <link rel = \" stylesheet \" href = \" default.css \" />\n";
    html << "  <style>\n";
    html << "* { font-family: Arial, Helvetica, sans-serif; }\n";
    html << "mark.red { color:#ff0000; background: none; }\n";
    html << "mark.blue { color:#0000A0; background: none; }\n";
    html << "  </style>\n";
    html << "</head>\n";

    html << "<body>\n";

    html << "<h1>" << input_video_metadata.name << "</h1>\n";

    html << "<img src=\"" << thumbnail_file << "\" alt=\"Preview\" width=\"800\">\n ";

    html << "<h2>Parameters</h2>\n";
    if (input_video_metadata.knownFocalLengthXPixels)
        html << "<p> Calculation uses known focal length of " << input_video_metadata.knownFocalLengthXPixels.value() << " x pixels.</p>\n";
    else
        html << "<p> Calculation run with unknown focal length.</p>\n";

    html << "<h2>Camera Intrinsics</h2>\n";

    //Pixel aspect ratio used : 1.5064(ground truth 1.56) - <10 % difference>
    //Focal length x pixels calculated : 1307.2(ground truth 1200) - <45 % difference>
    //Focal length mm calculated : 15.9996(ground truth 1200) - <45 % difference>

    // Pixel aspect ratio
    html << "<p> Pixel aspect ratio used: " << input_video_metadata.pixel_aspect_ratio_used;
    if (input_video_metadata.ground_truth_pixel_aspect_ratio()) {
        double diff_percent = percent_difference(input_video_metadata.ground_truth_pixel_aspect_ratio().value(),
                                                 input_video_metadata.pixel_aspect_ratio_used);
        if (diff_percent > par_percent_error_trigger)
            html << " (ground truth " << input_video_metadata.ground_truth_pixel_aspect_ratio().value() << " - <mark class=\"red\">  " << diff_percent << "\% difference</mark>)</p>\n";
        else
            html << " (ground truth " << input_video_metadata.ground_truth_pixel_aspect_ratio().value() << " - " << diff_percent << "\% difference)</p>\n";
    }
    else
        html << ".</p>\n";

    // Focal length x pixels
    html << "<p> Focal length x pixels calculated: " << calculated_focal_length_x_pixels;
    if (input_video_metadata.ground_truth_focal_length_x_pixels()) {
        double diff_percent = percent_difference(input_video_metadata.ground_truth_focal_length_x_pixels().value(),
                                                 calculated_focal_length_x_pixels);
        if (diff_percent > focal_percent_error_trigger)
            html << " (ground truth " << input_video_metadata.ground_truth_focal_length_x_pixels().value() << " - <mark class=\"red\">  " << diff_percent << "\% difference</mark>)</p>\n";
        else
            html << " (ground truth " << input_video_metadata.ground_truth_focal_length_x_pixels().value() << " - " << diff_percent << "\% difference)</p>\n";
    }
    else
        html << ".</p>\n";

    // Focal length mm
    if (input_video_metadata.calculated_focal_length_mm(calculated_focal_length_x_pixels)) {
        html << "<p> Focal length mm calculated: " << input_video_metadata.calculated_focal_length_mm(calculated_focal_length_x_pixels).value();
        if (input_video_metadata.groundTruthFocalLengthMM) {
            double diff_percent = percent_difference(input_video_metadata.groundTruthFocalLengthMM.value(),
                                                     input_video_metadata.calculated_focal_length_mm(calculated_focal_length_x_pixels).value());
            if (diff_percent > focal_percent_error_trigger)
                html << " (ground truth " << input_video_metadata.groundTruthFocalLengthMM.value() << " - <mark class=\"red\">  " << diff_percent << "\% difference</mark>)</p>\n";
            else
                html << " (ground truth " << input_video_metadata.groundTruthFocalLengthMM.value() << " - " << diff_percent << "\% difference)</p>\n";
        }
        else
            html << ".</p>\n";
    }

    html << "<h2> Map</h2>\n";
    // Solved/unsolved cameras
    html << "<p> " << solved_frame_count << " cameras created from the " << total_frames() << " frames of video";
    if (unsolved_frame_count!=0)
        html << " - <mark class=\"red\"> \<" << unsolved_frame_count << " frames not tracked\></mark></p>\n";
    html << "<p> " << num_points << " 3D points</p>\n";
    html << "<p> Initialisation Frames: {" << to_string(initialisation_frames) << "}</p>\n";

    // Timing
    html << "<h2> Timing</h2>\n";
    double fps = (double(total_frames()) / track_timings.total_time_sec());
    html << "<p> Total tracking time:" << track_timings.total_time_sec() << " sec (" << fps << "fps)</p>\n ";
    html << "<p> Forward mapping: " << track_timings.forward_mapping << ".</p>\n";
    html << "<p> Backward mapping: " << track_timings.backward_mapping << ".</p>\n";
    html << "<p> Loop Closing: " << track_timings.loop_closing << ".</p>\n";
    html << "<p> Optimisation: " << track_timings.optimisation << ".</p>\n";
    html << "<p> Final Tracking: " << track_timings.tracking << ".</p>\n";

    html << "</body></html>";

    myfile << html.str();
    myfile.close();
}


void metrics::save_html_overview(std::string_view const& filename,
                                 std::list<track_test_info> const& track_test_info_list) {
    std::ofstream myfile;
    myfile.open(filename.data());

    std::stringstream html;


	html << "<!DOCTYPE html> \n";
    html << "<html> \n";
    html << "<head> \n";
    html << "    <meta charset='utf-8' /> \n";
    html << "    <title>Camera Tracking Evaluation Run Overview</title> \n";
    html << "    <link rel=\" stylesheet \" href=\" default.css \" /> \n";
    html << "	<style>\n";
    html << "* { font-family: Arial, Helvetica, sans-serif; }\n";
    html << "#wrap > div { overflow:hidden; }\n";

    html << "#testruns warn { \n";
    html << "font-size:12px; \n";
//    html << "    color:#FF6A00; \n";
    html << "    font-style:normal; \n";
    html << "	display:inline-block; \n";
    html << "}	\n";
    html << "#testruns error { \n";
    html << "font-size:12px; \n";
    html << "    color:#FF0000; \n";
    html << "    font-style:normal; \n";
    html << "	display:inline-block; \n";
    html << "}	\n";
    html << "#testruns pass { \n";
    html << "    font-style:normal; \n";
    html << "    background:#cfcfcf; \n";
    html << "    margin-right:10px; \n";
    html << "    display:inline-block; \n";
    html << "    width:32px; \n";
    html << "    padding:0 10px; \n";
    html << "    border-radius:15px; \n";
    html << "    -moz-border-radius:15px; \n";
    html << "    -webkit-border-radius:15px; \n";
    html << "}\n";
    html << "#testruns fail { \n";
    html << "    font-style:normal; \n";
    html << "    background:#cfcfcf;\n";
    html << "    margin-right:10px; \n";
    html << "    display:inline-block; \n";
    html << "    width:32px; \n";
    html << "    padding:0 10px; \n";
    html << "    border-radius:15px; \n";
    html << "    -moz-border-radius:15px; \n";
    html << "    -webkit-border-radius:15px; \n";
    html << "}\n";
    html << "#testruns text { \n";
    html << "    font-style:normal; \n";
    html << "    display:inline-block; \n";
    html << "} 		\n";

    html << ".table {\n";
    html << "  display: table;\n";
    html << "}\n";

    html << ".row {\n";
    html << "  display: table-row;\n";
    html << "}\n";

    html << ".cell {\n";
    html << "  display: table-cell;\n";
    html << "  padding: 10px;\n";
    html << "}\n";

    html << ".row:hover {\n";
    html << "  background-color: #cccccc;\n";
    html << "}\n";

    //html << "h1 { margin:20; padding:20; font-size:40px; } \n";
    //html << "h1   {color: blue;}\n";
    html << "    h1    {color: #0088CC;}\n";
    html << "    p    {color: #0088CC;}\n";
    html << "		</style>\n";

    html << "<link href = \"https://stackpath.bootstrapcdn.com/twitter-bootstrap/2.3.2/css/bootstrap-combined.min.css\" rel=\"stylesheet\" type = \"text/css\" />\n ";



    html << "	</head> \n";

    html << "	<body> \n";

    html << "		<h1>Camera tracking evaluation run</h1> \n";

    html << "		<p>  Test run id: b6dafc8e-6c2b-478b-8dfe-55df227e8739 </p> \n";


    int pass_count(0), warn_count(0), fail_count(0);
    for (auto const& test_info : track_test_info_list) {
        if (test_info.m->max_problem_level() == std::nullopt)
            ++pass_count;
        if (test_info.m->max_problem_level() == tracking_problem_warning)
            ++warn_count;
        if (test_info.m->max_problem_level() == tracking_problem_fatal)
            ++fail_count;
    }

    html << "		<p>  Pass " << pass_count << ", warning " << warn_count << ", fail " << fail_count << "</p> \n";

    html << "<hr>\n";

    html << "<div id=\"testruns\" role=\"grid\" class=\"table\">\n";

    for (auto const& test_info : track_test_info_list) {
        metrics const* m(test_info.m);
        bool fail(m->solved_frame_count == 0);
        double fps = (double(m->total_frames()) / m->track_timings.total_time_sec());
        std::string style = fail ? "style=\"background-color: #dc8c8c; .hover:background-color: #dc8c8c;\"" : "";

        html << "  <a role=\"row\" class=\"row\" href=\"" << test_info.html_filename << "\"" << style << ">\n";
        html << "    <div role=\"gridcell\" class=\"cell\">\n";
        html << "      " << (fail ? "<fail>Fail</fail>" : "<pass>Pass</pass>") << "<img src=\"" << test_info.thumbnail_filename << "\" width=\"60\"><text>&nbsp;" << m->input_video_metadata.name << "</text>\n";
        html << "    </div>\n";
        html << "    <div role=\"gridcell\" class=\"cell\">\n";
        html << "      \n";
        html << "    </div>\n";
        html << "    <div role=\"gridcell\" class=\"cell\">\n";
        html << "      <text>" << m->total_frames() << " frames, " << m->track_timings.total_time_sec() << " sec (" << fps << "fps)</text>\n ";
        html << "    </div>\n";
        html << "    <div role=\"gridcell\" class=\"cell\">\n";
        html << "      <text>" << m->num_points << " points</text>\n";
        html << "    </div>\n";
        html << "    <div role=\"gridcell\" class=\"cell\">\n";
        html << "      ";
        auto problems = m->problems();
        for (auto const& problem : problems)
            if (problem.second==tracking_problem_warning)
               html << "<warn>&lt;" << problem.first << "&gt;</warn>";
            else
               html << "<error>&lt;" << problem.first << "&gt;</error>";
        html << "\n";
        html << "    </div>\n";
        html << "  </a> \n";
    }

    html << "</div>\n";
    html << "	</body> \n";
    html << "	</html>\n";


    myfile << html.str();
    myfile.close();
}

metrics_copy::metrics_copy(metrics const& m)
: m_copy(m)
{
}

metrics const& metrics_copy::operator()() {
    return m_copy;
}

static void fill_dummy_metrics(int i, metrics *m)
{
    using namespace std;
    using opt = vector<optional<double>>;

    if (i >= 3)
        i = 3;

    int num(i + 1);

    // metrics
    m->calculated_focal_length_x_pixels = vector({898.7, 898.7, 1307.2})[i];
    m->solved_frame_count = vector({0, 100, 201})[i];
    m->unsolved_frame_count = vector({76, 5, 0})[i];
    m->num_points = num * 17735;
    m->initialisation_frames = vector<std::set<int>>({{13, 14}, {2, 23}, {0, 10}})[i];

    // input video metadata
    m->input_video_metadata.name = vector({"Hillshot7", "Cup Final", "Fishtank"})[i];// A name used in reporting
    m->input_video_metadata.filename = vector({"Hillshot7.mpg", "Cup Final.mpg", "Fishtank.mpg"})[i]; // Name of the filename stored locally (in testdata/Cam3D)
    m->input_video_metadata.gDriveID = vector({"Hillshot7", "Cup Final", "Fishtank"})[i];              // google drive id of the video
    m->input_video_metadata.video_width = 1920;
    m->input_video_metadata.video_height = 1080;
    m->input_video_metadata.start_frame = vector({0, 100, 0})[i];
    m->input_video_metadata.end_frame = vector({77, 201, 200})[i];
    m->input_video_metadata.pixel_aspect_ratio_used = vector({1.5064, 1.0, 1.5064})[i];
    m->input_video_metadata.knownFocalLengthXPixels = opt({1000.0, {}, {}})[i];
    m->input_video_metadata.groundTruthFocalLengthXPixels = opt({{}, {}, {}})[i];
    m->input_video_metadata.groundTruthFilmBackWidthMM = 23.5;
    m->input_video_metadata.groundTruthFilmBackHeightMM = 15.6;
    m->input_video_metadata.groundTruthFocalLengthMM = opt({11, 11, 16})[i];

    // timings
    m->track_timings.forward_mapping = vector({112, 123, 164})[i];
    m->track_timings.backward_mapping = vector({131, 141, 165})[i];
    m->track_timings.loop_closing = vector({0.4, 1.2, 0.0})[i];
    m->track_timings.optimisation = vector({12, 11, 16})[i];
    m->track_timings.tracking = vector({115, 126, 167})[i];

}

void metrics_html_test(std::string const& directory, std::array<std::string, 3> image_filenames)
{
    // create some dummy metrics
    std::array<std::unique_ptr<metrics_copy>, 3> metrics_array;
    std::list<metrics::track_test_info> track_test_info_list;

    metrics* the_metrics = metrics::get_instance();

    for (int i = 0; i < 3; ++i) {
        fill_dummy_metrics(i, the_metrics);
        metrics_array[i] = std::make_unique<metrics_copy>(*the_metrics);
        std::string html_filename = directory + "/report" + std::to_string(i + 1) + ".html ";
        track_test_info_list.push_back({&metrics_array[i].get()->operator()(), image_filenames[i], html_filename});
        the_metrics->save_html_report(html_filename, image_filenames[i]);
    }
    metrics::save_html_overview(directory + "/overview.html", track_test_info_list);
}





} // namespace stella_vslam_bfx
