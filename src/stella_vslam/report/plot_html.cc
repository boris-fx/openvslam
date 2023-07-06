#include "plot_html.h"

#include <iomanip>
#include <sstream>
#include <array>
#include <fstream>
#include <vector>
#include <list>

#include <stella_vslam/solve/fundamental_consistency.h> // just for the median function, which should be somewhere else

Graph::Graph(std::string x_label, std::string y_label, std::set<Curve> curves,
             axis_scaling x_axis_scaling, axis_scaling y_axis_scaling,
             std::optional<double> ground_truth_y)
: x_label(x_label), y_label(y_label), ground_truth_y(ground_truth_y)
, x_axis_scaling(x_axis_scaling), y_axis_scaling(y_axis_scaling)
{
    for (auto const& curve : curves)
        this->curves.insert({ curve.first, { curve_section(curve.second, 0) } });
}

struct LabelToValue {
    LabelToValue(std::string label, float value, int index)
        : label(label), value(value), index(index) {}
    LabelToValue()
        : value(0), index(-1) {}
    std::string label;
    float value;
    int index;
};

struct labelled_curve_section : public std::vector<LabelToValue>
{
    labelled_curve_section(std::vector<LabelToValue> const& points, std::optional<int> stage=std::nullopt) : std::vector<LabelToValue>(points), stage(stage) {}
    labelled_curve_section(size_t size, std::optional<int> stage = std::nullopt) : std::vector<LabelToValue>(size), stage(stage) {}
    std::optional<int> stage;
};

struct LabelledCurve {
    bool ghost = false; // No circles, and grey - e.g. indication of a ground truth value
    std::string name;
    //std::list<std::vector<LabelToValue>> dataValues;
    std::list<labelled_curve_section> labelled_curve_sections;
};

std::string floatLabelString(float v, int decimalPlaces)
{
    if (decimalPlaces > 0) {
        std::ostringstream str;
        str << std::setprecision(decimalPlaces) << std::fixed << v;
        return str.str();
    }
    else
        return std::to_string(int(v + 0.5f));
}

std::list<std::pair<float, std::string>> tickMarksForValueRange(float minVal, float maxVal, float tickCount)
{
   float range(maxVal - minVal);

   float tickSeparation(range / tickCount);
   float logSep = log10(tickSeparation);
   float flooredLogSep = floor(logSep);
   float scale = pow(10.0, flooredLogSep); // Nearest power of 10 below the tickSeparation
   //bool integerScale(scale>(1.0f-0.001f));
   //float scaledMin(minVal/scale);
   //float scaledMax(maxVal/scale);

   int s(1); // final tick separation is this * scale
   float div(tickSeparation / scale);
   if (div > 2.0f)
      s = 2;
   if (div > 5.0f)
      s = 5;
   float finalSeparation(float(s)*scale);

   std::list<std::pair<float, std::string>> ticks;
   float start = finalSeparation * floor(minVal / finalSeparation);
   float tick;
   for (tick = start; tick < maxVal; tick += finalSeparation)
      //if (integerScale)
      //   ticks.push_back( { tick, std::to_string(int(tick+0.5f)) } );
      //else
      ticks.push_back({ tick, floatLabelString(tick, -flooredLogSep) });

   // One extra tick after the max value
   //if (integerScale)
   //   ticks.push_back( { tick, std::to_string(int(tick+0.5f)) } );
   //else
   //   ticks.push_back( { tick, std::to_string(tick) } );
   ticks.push_back({ tick, floatLabelString(tick, -flooredLogSep) });

   //std::list<std::pair<float, std::string>> ticks;
   //int subsampling(1);
   //if (range>20) subsampling = 2;
   //if (range>50) subsampling = 5;
   //if (range>100) subsampling = 10;
   //if (range>200) subsampling = 20;
   //if (range>500) subsampling = 50;
   //for (int i=0; i<ceil(range+1e-6); ++i) {
   //   if (((i)%subsampling)==0) {
   //      ticks.push_back({i, std::to_string(i)});
   //   }
   //}

   return ticks;
}

float Hue_2_RGB(float v1, float v2, float vH) {
    if (vH < 0.0F)
        vH += 1.0F;
    if (vH > 1.0F)
        vH -= 1.0F;
    if ((6.0F * vH) < 1.0F)
        return (v1 + (v2 - v1) * 6.0F * vH);
    if ((2.0F * vH) < 1.0F)
        return (v2);
    if ((3.0F * vH) < 2.0F)
        return (v1 + (v2 - v1) * ((0.6666666666667F) - vH) * 6.0F);
    return (v1);
}

std::vector<float> getRGBFromHSL(const float HSL[3]) {
    std::vector<float> RGB_(3);
    if (HSL[1] == 0) {
        RGB_[0] = HSL[2];
        RGB_[1] = HSL[2];
        RGB_[2] = HSL[2];
    }
    else {
        float var_1, var_2;
        if (HSL[2] < 0.5)
            var_2 = HSL[2] * (1 + HSL[1]);
        else
            var_2 = (HSL[2] + HSL[1]) - (HSL[1] * HSL[2]);
        var_1 = 2.0 * HSL[2] - var_2;
        RGB_[0] = Hue_2_RGB(var_1, var_2, HSL[0] + (1.0 / 3.0));
        RGB_[1] = Hue_2_RGB(var_1, var_2, HSL[0]);
        RGB_[2] = Hue_2_RGB(var_1, var_2, HSL[0] - (1.0 / 3.0));
    }
    return RGB_;
}

inline float Sobol2(unsigned int n, unsigned int scramble) {
    const float OneMinusEpsilon = 0.9999999403953552f;
    for (unsigned int v = (unsigned int)(1 << 31); n != 0; n >>= 1, v ^= v >> 1)
        if (n & 0x1)
            scramble ^= v;
    return (std::min)(((scramble >> 8) & 0xffffff) / float(1 << 24), OneMinusEpsilon);
}
std::vector<std::vector<float>> generateColours(unsigned int n, float saturation, float lightness) {
    if (n == 0)
        return (std::vector<std::vector<float>>());

    // Just vary hue
    std::vector<float> hue(n);
    for (unsigned int i = 0; i < n; ++i)
        hue[i] = Sobol2(i, 0);

    std::vector<std::vector<float>> cols(n, std::vector<float>(3));
    float HSL[3] = {0.0f, saturation, lightness};
    for (unsigned int i = 0; i < n; ++i) {
        HSL[0] = hue[i];
        cols[i] = getRGBFromHSL(HSL);
    }

    return cols;
}

std::vector<std::pair<float, float>> get_x_axis_ranges(std::list<LabelledCurve> const& curves, bool split_x_axis, std::optional<double> const& hard_max_x)
{
    if (split_x_axis) {

        // Collect the curve sections by stage
        std::vector<std::list<labelled_curve_section>> stage_to_curve_section_list;
        for (auto const& curve : curves) {
            for (auto const& section : curve.labelled_curve_sections) {
                if (section.stage) {
                    int stage = section.stage.value();
                    if (stage_to_curve_section_list.size() < (stage + 1))
                        stage_to_curve_section_list.resize(stage + 1);
                    stage_to_curve_section_list[stage].push_back(section);
                }
            }
        }

        std::vector<std::pair<float, float>> result(stage_to_curve_section_list.size());

        for (int stage = 0; stage < stage_to_curve_section_list.size(); ++stage) {

            float minXValue(std::numeric_limits<float>::max()), maxXValue(0);

            for (auto const& section : stage_to_curve_section_list[stage]) {
                for (auto const& data_value : section) {
                    if (maxXValue < data_value.index)
                        maxXValue = data_value.index;
                    if (minXValue > data_value.index)
                        minXValue = data_value.index;
                }
            }

            //if (hard_max_x && maxXValue > hard_max_x.value())
            //maxXValue = hard_max_x.value();
            if (minXValue == std::numeric_limits<float>::max()) { // No data points
                minXValue = 0;
                maxXValue = 1;
            }
            if (minXValue == maxXValue) { // One data point
                minXValue -= 0.5f;
                maxXValue += 0.5f;
            }
            result[stage] = { minXValue, maxXValue };
        }


        //// Get the min/max x value
        //for (auto const& curve : curves) {
        //    for (auto const& data_value_set : curve.labelled_curve_sections) {
        //        float minXValue(std::numeric_limits<float>::max()), maxXValue(0);

        //        for (auto const& data_value : data_value_set) {
        //            if (maxXValue < data_value.index)
        //                maxXValue = data_value.index;
        //            if (minXValue > data_value.index)
        //                minXValue = data_value.index;
        //        }

        //        //if (hard_max_x && maxXValue > hard_max_x.value())
        //        //maxXValue = hard_max_x.value();
        //        if (minXValue == std::numeric_limits<float>::max()) { // No data points
        //            minXValue = 0;
        //            maxXValue = 1;
        //        }
        //        if (minXValue == maxXValue) { // One data point
        //            minXValue -= 0.5f;
        //            maxXValue += 0.5f;
        //        }

        //        result.push_back({ minXValue, maxXValue });
        //    }
        //}
        return result;
    }
    else {
        // Get the min/max x value
        float minXValue(std::numeric_limits<float>::max()), maxXValue(0);
        for (auto const& curve : curves) {
            for (auto const& data_value_set : curve.labelled_curve_sections) {
                for (auto const& data_value : data_value_set) {
                    if (maxXValue < data_value.index)
                        maxXValue = data_value.index;
                    if (minXValue > data_value.index)
                        minXValue = data_value.index;
                }
            }
        }
        if (hard_max_x && maxXValue > hard_max_x.value())
            maxXValue = hard_max_x.value();
        if (minXValue == std::numeric_limits<float>::max()) { // No data points
            minXValue = 0;
            maxXValue = 1;
        }
        if (minXValue == maxXValue) { // One data point
            minXValue -= 0.5f;
            maxXValue += 0.5f;
        }
        return { {minXValue, maxXValue} };
    }
}

std::pair<float, float> get_range_fraction(std::vector<std::pair<float, float>>& ranges, int i, float total_range_gap_fraction)
{
    if (ranges.size() == 1 && i == 0)
        return { 0.0f, 1.0f };
    if (i < 0 || i >= ranges.size())
        return { 0.0f, 0.0f };
    float range_sum(0), range_sum_before(0);
    for (int r = 0; r < ranges.size(); ++r) {
        range_sum += std::abs(ranges[r].second - ranges[r].first);
        if (r<i)
            range_sum_before += std::abs(ranges[r].second - ranges[r].first);
    }
    if (range_sum==0.0f)
        return { 0.0f, 0.0f };
    float this_range_fraction = std::abs(ranges[i].second - ranges[i].first) / range_sum;
    float range_fraction_before = range_sum_before / range_sum;

    // Rescale to the fraction remaining after including gaps
    this_range_fraction *= (1.0f - total_range_gap_fraction);
    range_fraction_before *= (1.0f - total_range_gap_fraction);
    float gap_fraction_before = total_range_gap_fraction * float(i) / (float)((int)ranges.size() - 1);
    return { range_fraction_before + gap_fraction_before, range_fraction_before + gap_fraction_before + this_range_fraction };
}

void write_graph_as_svg_impl(std::stringstream& svg, std::string_view yLabel, std::string_view xLabel, std::list<LabelledCurve> const& curves,
                             bool linearXLabels, int graphWidth, int graphHeight, bool drawVertexCircles,
                             bool split_x_axis, std::optional<double> hard_max_x, std::optional<double> hard_max_y, bool no_min_0_y) {
   int x_axis_label_size(200); // Size of text xLabel
   int labelSize(200); // size of text labelled_curve_sections[i].label - todo calculate based on string length - maybe use 'textLength' attribute to force size
   int legendWidth(400);

   bool no_data_points(true);
   for (auto const& curve : curves)
       for (auto const& data_value_set : curve.labelled_curve_sections)
           if (!data_value_set.empty() && !curve.ghost) {
               no_data_points = false;
               break;
           }
   if (no_data_points) {
       svg << "<svg height=\"30\" width=\"" << graphWidth << "\">" << std::endl;
       svg << "<text x = \"0\" y = \"15\">" << "Graph '" << xLabel << "' vs '" << yLabel << "' has no data points." << "</text>" << std::endl;
       svg << "Sorry, your browser does not support inline SVG." << std::endl;
       svg << "</svg>" << std::endl;
       return;
   }

   int width(graphWidth), height(graphHeight);

   svg << "<svg height=\"" << height << "\" width=\"" << width + legendWidth << "\">" << std::endl;

   // Horizontal axis (x-value 0, 1, 2, 3...)
   int xCanvas0(30);
   int yCanvas0(linearXLabels ? height - 50 : height - labelSize); // bottom
   float xCanvasInc(0);
   float yCanvasInc(0);
   float xStartValue(0);
   float yStartValue(0);
   int yCanvasEnd(0);
   std::map<std::optional<int>, float> x_value_start_for_stage;
   std::map<std::optional<int>, float> x_canvas_start_for_stage;
   std::map<std::optional<int>, float> x_canvas_end_for_stage;
   std::map<std::optional<int>, float> x_canvas_inc_for_stage;

   if (!linearXLabels)
   {
      std::set<int> allTimeValues;
      std::map<int, std::string> timeValueToLabel;
      for (auto const& curve : curves)
         for (auto const& data_value_set : curve.labelled_curve_sections) {
             for (auto const& data_value : data_value_set) {
                 allTimeValues.insert(data_value.index);
                 timeValueToLabel[data_value.index] = data_value.label;
             }
         }
      std::vector<std::string> labels(timeValueToLabel.size());
      int iLabel(0);
      for (auto const& label : timeValueToLabel) {
         labels[iLabel] = label.second;
         ++iLabel;
      }

      int xCanvasEnd(width - x_axis_label_size);
      svg << "<text x=\"" << xCanvasEnd + 1 << "\" y=\"" << yCanvas0 << "\" fill=\"black\" >" << xLabel << "</text>" << std::endl;
      svg << "<line x1=\"" << xCanvas0 << "\" y1=\"" << yCanvas0 << "\" x2=\"" << xCanvasEnd << "\" y2=\"" << yCanvas0 << "\" style=\"stroke:rgb(0,0,0);stroke-width:2\" />" << std::endl;
      xCanvasInc = (float(xCanvasEnd - xCanvas0) / float(labels.size() + 1));
      int subsampling(1);
      if (labels.size() > 20) subsampling = 2;
      if (labels.size() > 50) subsampling = 5;
      if (labels.size() > 100) subsampling = 10;
      if (labels.size() > 200) subsampling = 20;
      if (labels.size() > 500) subsampling = 50;
      std::list<std::pair<float, std::string>> ticks; // value markers on the horizontal axis
      for (int i = 0; i < labels.size(); ++i) {
         if (i == 0 || ((i + 1) % subsampling) == 0) {
            int ep(i + 1);
            float l(xCanvas0 + float(ep)*xCanvasInc);
            ticks.push_back(std::pair<float, std::string>(l, labels[i]));
         }
      }
      for (auto const& tick : ticks) {
         float textOffset(-4); // x offset
         //if (tick.second.size()>=1) textOffset = -8;
         //if (tick.second.size()>=2) textOffset = -12;
         //if (tick.second.size()>=3) textOffset = -16;

         svg << "<line x1=\"" << tick.first << "\" y1=\"" << yCanvas0 << "\" x2=\"" << tick.first << "\" y2=\"" << yCanvas0 + 5 << "\" style=\"stroke:rgb(0,0,0);stroke-width:1\" />" << std::endl;
         float tx(tick.first + textOffset);
         float ty(yCanvas0 + 7);
         svg << "<text x=\"" << tx << "\" y=\"" << ty << "\" fill=\"black\" transform=\"rotate(90 " << tx << "," << ty << ")\">" << tick.second << "</text>" << std::endl;
      }
   }
   else // linearLabels
   {
      float approxTickCountX(20);

      

      std::vector<std::pair<float, float>> x_axis_ranges = get_x_axis_ranges(curves, split_x_axis, hard_max_x);
      float total_range_gap_fraction(0.03f);


      x_value_start_for_stage[std::nullopt] = 0.0f;
      x_canvas_start_for_stage[std::nullopt] = xCanvas0;
      x_canvas_end_for_stage[std::nullopt] = width - x_axis_label_size;
      x_canvas_inc_for_stage[std::nullopt] = 1.0f;

      for (int i = 0; i < x_axis_ranges.size(); ++i) {
          float minXValue = x_axis_ranges[i].first;
          float maxXValue = x_axis_ranges[i].second;
          std::pair<float, float> range_fraction = get_range_fraction(x_axis_ranges, i, total_range_gap_fraction);
          float range_span(range_fraction.second - range_fraction.first);

          std::list<std::pair<float, std::string>> ticks = tickMarksForValueRange(minXValue, maxXValue, approxTickCountX * range_span);

          int xCanvasEnd(width - x_axis_label_size);
          svg << "<text x=\"" << xCanvasEnd + 1 << "\" y=\"" << yCanvas0 << "\" fill=\"black\" >" << xLabel << "</text>" << std::endl;

          float section0(xCanvas0 + float(xCanvasEnd- xCanvas0)* range_fraction.first);
          float section1 = section0 + float(xCanvasEnd - xCanvas0) * range_span;
          svg << "<line x1=\"" << section0 << "\" y1=\"" << yCanvas0 << "\" x2=\"" << section1 << "\" y2=\"" << yCanvas0 << "\" style=\"stroke:rgb(0,0,0);stroke-width:2\" />" << std::endl;
          xCanvasInc = (float(section1 - section0) / ((maxXValue - minXValue) * 1.02f));
          xStartValue = minXValue - 0.01f * (maxXValue - minXValue);

          if (i % 2 == 1) {
              float old_section0(section0);
              section0 = section1;
              section1 = old_section0;
              xCanvasInc = -xCanvasInc;
          }

          x_value_start_for_stage[i] = xStartValue;
          x_canvas_start_for_stage[i] = section0;
          x_canvas_end_for_stage[i] = section1;
          x_canvas_inc_for_stage[i] = xCanvasInc;

          for (auto const& tick : ticks) {
              float textOffset(-4); // x offset

              float xPos(section0 + (tick.first - xStartValue) * xCanvasInc);

              svg << "<line x1=\"" << xPos << "\" y1=\"" << yCanvas0 << "\" x2=\"" << xPos << "\" y2=\"" << yCanvas0 + 5 << "\" style=\"stroke:rgb(0,0,0);stroke-width:1\" />" << std::endl;
              float tx(xPos + textOffset);
              float ty(yCanvas0 + 7);
              svg << "<text x=\"" << tx << "\" y=\"" << ty << "\" fill=\"black\" transform=\"rotate(90 " << tx << "," << ty << ")\">" << tick.second << "</text>" << std::endl;
          }
      }
   }

   // Vertical axis
   {
      int approxTickCountY(10);

      yCanvasEnd = yLabel.empty() ? 0 : 25; // top
      svg << "<text x=\"" << xCanvas0 - 25 << "\" y=\"" << yCanvasEnd - 10 << "\" fill=\"black\" >" << yLabel << "</text>" << std::endl;
      svg << "<line x1=\"" << xCanvas0 << "\" y1=\"" << yCanvas0 << "\" x2=\"" << xCanvas0 << "\" y2=\"" << yCanvasEnd << "\" style=\"stroke:rgb(0,0,0);stroke-width:2\" />" << std::endl;

      // Get the max y value
      float minYValue(std::numeric_limits<float>::max()), maxYValue(0);
      for (auto const& curve : curves)
          for (auto const& data_value_set : curve.labelled_curve_sections) {
              for (auto const& data_value : data_value_set) {
                  if (maxYValue < data_value.value)
                      maxYValue = data_value.value;
                  if (minYValue > data_value.value)
                      minYValue = data_value.value;
              }
          }
      if (!no_min_0_y)
          minYValue = 0;

      if (hard_max_y && maxYValue > hard_max_y.value())
          maxYValue = hard_max_y.value();
      if (no_data_points) {
          minYValue = 0.0f;
          maxYValue = 1.0f;
      }
      if (minYValue == maxYValue) { // One data point
          minYValue -= 0.5f;
          maxYValue += 0.5f;
      }

      std::list<std::pair<float, std::string>> ticks = tickMarksForValueRange(minYValue, maxYValue, approxTickCountY);

      yCanvasInc = (float(yCanvasEnd - yCanvas0) / (float(maxYValue - minYValue)*1.02f));
      yStartValue = minYValue - 0.01f * (maxYValue - minYValue);

      for (auto const& tick : ticks) {

         float textOffset(-10);
         if (tick.second.size() >= 2) textOffset = -18;
         if (tick.second.size() >= 3) textOffset = -26;
         if (tick.second.size() >= 4) textOffset = -30;

         float tickFirst(yCanvas0 + (tick.first - yStartValue)*yCanvasInc);
         //float xPos(section0 + (tick.first - xStartValue) * xCanvasInc);

         svg << "<line x1=\"" << xCanvas0 - 5 << "\" y1=\"" << tickFirst << "\" x2=\"" << xCanvas0 << "\" y2=\"" << tickFirst << "\" style=\"stroke:rgb(0,0,0);stroke-width:1\" />" << std::endl;
         svg << "<text x=\"" << xCanvas0 + textOffset - 2 << "\" y=\"" << tickFirst + 5 << "\" fill=\"black\" >" << tick.second << "</text>" << std::endl;
      }
   }

   std::vector<std::array<int, 3>> rgb;
   {
      float saturation(0.5f), lightness(0.5f);
      std::vector<std::vector<float> > floatColours = generateColours(curves.size(), saturation, lightness);
      rgb.resize(floatColours.size());
      for (int c = 0; c < floatColours.size(); ++c)
         for (int i = 0; i < 3; ++i)
            rgb[c][i] = int(255.0f * floatColours[c][i]);
   }

   // Plot the curves (polylines)
   int c(0);
   for (auto const& curve : curves) {

       for (auto const& data_value_set : curve.labelled_curve_sections) {

           auto stage = data_value_set.stage;
           float x_value_start = x_value_start_for_stage[stage];
           float x_canvas_start = x_canvas_start_for_stage[stage];
           float x_canvas_end = x_canvas_end_for_stage[stage];
           float x_canvas_inc = x_canvas_inc_for_stage[stage];

           if (curve.ghost) {
               rgb[c] = { 128, 128, 128 }; // grey
           }

           // Collect the curve segments's point positions in canvas space
           std::list<std::pair<float, float>> xyList;
           if (curve.ghost) {
               if (!data_value_set.empty()) {
                   xyList.push_back({ xCanvas0,                  yCanvas0 + (data_value_set.begin()->value - yStartValue) * yCanvasInc });
                   xyList.push_back({ width - x_axis_label_size, yCanvas0 + (data_value_set.begin()->value - yStartValue) * yCanvasInc });
               }
           }
           else {
               for (auto const& data_value : data_value_set) {
                   //xyList.push_back({ xCanvas0 + (data_value.index - xStartValue) * xCanvasInc,
                   float x = x_canvas_start + (data_value.index - x_value_start) * x_canvas_inc;
                   if (!std::isfinite(x))
                       x = x_canvas_end + 1000.0f;// 100000.0f;//  std::numeric_limits<float>::max();
                   float y = yCanvas0 + (data_value.value - yStartValue) * yCanvasInc;
                   if (!std::isfinite(y))
                       y = yCanvasEnd - 1000.0f; // 100000.0f;//std::numeric_limits<float>::max();
                   xyList.push_back({ x, y });
               }
           }

           // Plot the curve segment as a polyline
           svg << "<polyline points=\"";
           for (auto const& xy : xyList)
               svg << xy.first << "," << xy.second << " ";
           int stroke_width(curve.ghost ? 1 : 3);
           svg << "\" style=\"fill:none;stroke:rgb(" << rgb[c][0] << "," << rgb[c][1] << "," << rgb[c][2] << ");stroke-width:" << stroke_width << "\" />" << std::endl;

           // Draw a circle on each vertex of the curve
           if (drawVertexCircles && !curve.ghost)
               for (auto const& xy : xyList)
                   svg << "<circle cx = \"" << xy.first << "\" cy = \"" << xy.second << "\" r = \"3\" fill = \"rgb(" << rgb[c][0] << "," << rgb[c][1] << "," << rgb[c][2] << ")\" />" << std::endl;
       }
       ++c;
   }

   // Add a legend
   int y(30);
   c = 0;
   for (auto const& curve : curves) {
       svg << "<text x=\"" << width << "\" y=\"" << y << "\" style=\"fill:rgb(" << rgb[c][0] << "," << rgb[c][1] << "," << rgb[c][2] << ");\">" << curve.name << "</text>" << std::endl;
      ++c;
      y += 30;
   }

   svg << "Sorry, your browser does not support inline SVG." << std::endl;
   svg << "</svg>" << std::endl;
}

double graph_median(std::set<SplitCurve> const& curves)
{
    // Take the largest median of each curve
    double largest(0.0);
    for (auto const& curve : curves) {
        std::vector<double> y_values;
        for (auto const& data_point_set : curve.second)
            for (auto const& data_point : data_point_set)
                y_values.push_back(data_point.second);
        double median = stella_vslam_bfx::median(y_values);
        if (largest < median)
            largest = median;
    }
    return largest;
}

void write_graph_as_svg(std::stringstream& svg, Graph const& graph)
{
    bool split_x_axis(graph.x_axis_scaling.behaviour == range_behaviour::split_by_stage);

    //auto [xLabel, yLabel, curves, y_axis_scaling, ground_truth_y] = graph;
    std::list<LabelledCurve> labelled_curves;

    unsigned int vertex_count(0);
    for (auto const& curve : graph.curves) {
        LabelledCurve labelled_curve;
        labelled_curve.name = curve.first;

        //labelled_curve.labelled_curve_sections.resize(curve.second.size());
        //int i(0);
        //for (auto const& vertex : curve.second) {
        //    labelled_curve.labelled_curve_sections[i].index = int(vertex.first + 0.5);
        //    labelled_curve.labelled_curve_sections[i].value = (float)vertex.second;
        //    //labelled_curve.labelled_curve_sections[i].label = std::string(); // the x-values can also have text labels
        //    ++i;
        //}

        for (auto const& vertex_set : curve.second) {
            labelled_curve_section point_set(vertex_set.size(), split_x_axis ? vertex_set.stage : 0);
            int i(0);
            for (auto const& vertex : vertex_set) {
                point_set[i].index = int(vertex.first + 0.5);
                point_set[i].value = (float)vertex.second;
                //point_set[i].label = std::string(); // the x-values can also have text labels
                ++i;
            }
            labelled_curve.labelled_curve_sections.push_back(point_set);
        }
        vertex_count += labelled_curve.labelled_curve_sections.size();

        labelled_curves.push_back(labelled_curve);
    }

    if (graph.ground_truth_y) {
        std::set<float> x_values;
        for (auto const& curve : graph.curves)
            for (auto const& data_point_set : curve.second)
                for (auto const& vertex : data_point_set)
                    x_values.insert(vertex.first);
        if (!x_values.empty()) {
            LabelledCurve ghost_curve;
            ghost_curve.ghost = true;
            ghost_curve.labelled_curve_sections = { labelled_curve_section({ LabelToValue("", graph.ground_truth_y.value(), *x_values.begin()),
                                                                LabelToValue("", graph.ground_truth_y.value(), *x_values.rbegin()) }, std::nullopt ) } ;
            labelled_curves.push_back(ghost_curve);
        }
    }
    bool linearXLabels(true);

    std::optional<double> hard_max_x;
    if (graph.x_axis_scaling.behaviour == range_behaviour::hard_max)
        hard_max_x = graph.x_axis_scaling.max;
//    if (graph.x_axis_scaling.behaviour == range_behaviour::max_from_median)
  //      hard_max_x = 1.5 * graph_median(graph.curves);
    

    std::optional<double> hard_max_y;
    if (graph.y_axis_scaling.behaviour == range_behaviour::hard_max)
        hard_max_y = graph.y_axis_scaling.max;
    if (graph.y_axis_scaling.behaviour == range_behaviour::max_from_median)
        hard_max_y = 1.5 * graph_median(graph.curves);

    write_graph_as_svg_impl(svg, graph.y_label, graph.x_label, labelled_curves, linearXLabels, 1000, 500, true, split_x_axis, hard_max_x, hard_max_y, graph.y_axis_scaling.behaviour == range_behaviour::no_min_0);
}


html_file::html_file(std::string_view const& filename) {
    myfile.open(filename.data());
    html << "<!DOCTYPE html><html><head></head><body>";
}

html_file::~html_file()
{
    html << "</body></html>";
    myfile << html.str();
    myfile.close();
}

void write_graphs_html(std::string_view const& filename, std::set<Graph> graphs)
{
    html_file html(filename);
    for (auto const& graph : graphs)
        write_graph_as_svg(html.html, graph);
    html << "<p>text</p>";
}

bool disable_all_html_graph_export() {
   return true;
}

std::list<curve_section> get_sample_graphs() {

    curve_section a = curve_section({
    {0, 46},
    { 1, 45 },
    { 2, 45 },
    { 3, 45 },
    { 4, 45 },
    { 5, 45 },
    { 6, 45 },
    { 7, 44 },
    { 8, 44 },
    { 9, 44 },
    { 10, 44 },
    { 11, 43 },
    { 12, 43 },
    { 13, 43 },
    { 14, 43 },
    { 15, 43 },
    { 16, 43 },
    { 17, 43 },
    { 18, 42 },
    { 19, 42 },
    { 20, 42 },
    { 21, 42 },
    { 22, 43 },
    { 23, 43 },
    { 24, 42 },
    { 25, 42 },
    { 26, 42 },
    { 27, 42 },
    { 28, 42 },
    { 29, 42 },
    { 30, 42 },
    { 31, 42 },
    { 32, 42 },
    { 33, 42 },
    { 34, 42 },
    { 35, 42 },
    { 36, 42 },
    { 37, 41 },
    { 38, 41 },
    { 39, 41 },
    { 40, 41 },
    { 41, 41 },
    { 42, 41 },
    { 43, 41 },
    { 44, 41 },
    { 45, 41 },
    { 46, 41 },
    { 47, 41 },
    { 48, 41 },
    { 49, 41 },
    }, 1);


    curve_section b = curve_section({
    {54, 2},
    { 55, 3 },
    { 56, 4 },
    { 57, 5 },
    { 58, 5 },
    { 59, 5 },
    { 60, 5 },
    { 61, 5 },
    { 62, 6 },
    { 63, 6 },
    { 64, 7 },
    { 65, 7 },
    { 66, 7 },
    { 67, 7 },
    { 68, 8 },
    { 69, 8 },
    { 70, 8 },
    { 71, 8 },
    { 72, 9 },
    { 73, 9 },
    { 74, 9 },
    { 75, 9 },
    { 76, 9 },
    { 77, 9 },
    { 78, 9 },
    { 79, 10 },
    { 80, 10 },
    { 81, 10 },
    { 82, 10 },
    { 83, 10 },
    { 84, 10 },
    { 85, 11 },
    { 86, 12 },
    { 87, 12 },
    { 88, 12 },
    { 89, 12 },
    { 90, 12 },
    { 91, 12 },
    { 92, 12 },
    { 93, 12 },
    { 94, 12 },
    { 95, 12 },
    { 96, 13 },
    { 97, 13 },
    { 98, 13 },
    { 99, 13 },
    { 100, 14 },
    { 101, 14 },
    { 102, 14 },
    { 103, 14 },
    { 104, 14 },
    { 105, 14 },
    { 106, 15 },
    { 107, 15 },
    { 108, 15 },
    { 109, 15 },
    { 110, 15 },
    { 111, 15 },
    { 112, 15 },
    { 113, 15 },
    { 114, 16 },
    { 115, 16 },
    { 116, 16 },
    { 117, 16 },
    { 118, 16 },
    { 119, 16 },
    { 120, 16 },
    { 121, 16 },
    { 122, 16 },
    { 123, 16 },
    { 124, 16 },
    { 125, 17 },
    { 126, 17 },
    { 127, 17 },
    { 128, 17 },
    { 129, 17 },
    { 130, 18 },
    { 131, 18 },
    { 132, 18 },
    { 133, 18 },
    { 134, 18 },
    { 135, 18 },
    { 136, 18 },
    { 137, 18 },
    { 138, 18 },
    { 139, 18 },
    { 140, 19 },
    { 141, 19 },
    { 142, 19 },
    { 143, 19 },
    { 144, 19 },
    { 145, 19 },
    { 146, 19 },
    { 147, 19 },
    { 148, 19 },
    { 149, 19 },
    { 150, 19 },
    { 151, 20 },
    { 152, 20 },
    { 153, 20 },
    { 154, 20 },
    { 155, 20 },
    { 156, 20 },
    { 157, 20 },
    { 158, 20 },
    { 159, 20 },
    { 160, 20 },
    { 161, 21 },
    { 162, 21 },
    { 163, 21 },
    { 164, 22 },
    { 165, 22 },
    { 166, 22 },
    { 167, 22 },
    { 168, 22 },
    { 169, 22 },
    { 170, 22 },
    { 171, 22 },
    { 172, 22 },
    { 173, 22 },
    { 174, 23 },
    { 175, 23 },
    { 176, 23 },
    { 177, 23 },
    { 178, 23 },
    { 179, 24 },
    { 180, 24 },
    { 181, 24 },
    { 182, 24 },
    { 183, 24 },
    { 184, 24 },
    { 185, 24 },
    { 186, 24 },
    { 187, 25 },
    { 188, 25 },
    { 189, 25 },
    { 190, 25 },
    { 191, 25 },
    { 192, 25 },
    { 193, 25 },
    { 194, 25 },
    { 195, 26 },
    { 196, 26 },
    { 197, 26 },
    { 198, 26 },
    { 199, 26 },
    { 200, 27 },
    { 201, 27 },
    { 202, 27 },
    { 203, 26 },
    { 204, 26 },
    { 205, 26 },
    { 206, 26 },
    { 207, 26 },
    { 208, 27 },
    { 209, 27 },
    { 210, 27 },
    { 211, 27 },
    { 212, 27 },
    { 213, 27 },
    { 214, 28 },
    { 215, 28 },
    { 216, 28 },
    { 217, 28 },
    { 218, 29 },
    { 219, 29 },
    { 220, 29 },
    { 221, 29 },
    { 222, 29 },
    { 223, 29 },
    { 224, 29 },
    { 225, 30 },
    { 226, 30 },
    { 227, 30 },
    { 228, 31 },
    { 229, 31 },
    { 230, 31 },
    { 231, 31 },
    { 232, 31 },
    { 233, 31 },
    { 234, 32 },
    { 235, 32 },
    { 236, 32 },
    { 237, 32 },
    { 238, 32 },
    { 239, 32 },
    { 240, 33 },
    { 241, 33 },
    { 242, 33 },
    { 243, 33 },
    { 244, 33 },
    { 245, 33 },
    { 246, 34 },
    { 247, 34 },
    { 248, 34 },
    { 249, 34 },
    { 250, 34 },
    { 251, 35 },
    { 252, 35 },
    { 253, 35 },
    { 254, 35 },
    { 255, 35 },
    { 256, 36 },
    { 257, 36 },
    { 258, 36 },
    { 259, 36 },
    { 260, 37 },
    { 261, 37 },
    { 262, 37 },
    { 263, 38 },
    { 264, 38 },
    { 265, 38 },
    { 266, 38 },
    { 267, 38 },
    { 268, 39 },
    { 269, 39 },
    { 270, 39 },
    { 271, 39 },
    { 272, 39 },
    { 273, 40 },
    { 274, 40 },
    { 275, 40 },
    { 276, 41 },
    { 277, 41 }
    }, 0);

    return { a, b };
}

std::list<curve_section> get_sample_graphs_2() {

    // non-planar
    curve_section a = curve_section({
    { 1, 0.331036 },
    { 2, 0.548319 },
    { 3, 0.635865 },
    { 4, 0.805998 },
    { 5, 0.638672 },
    { 9, 0.670777 },
    { 10, 0.863175 },
    { 11, 0.819828 },
    { 12, 0.935065 },
    { 13, 1.11409 },
    { 17, 0.46498 },
    { 18, 0.816095 },
    { 19, 0.872364 },
    { 20, 0.870318 },
    { 21, 1.25207 },
    { 22, 1.53 },
    { 23, 1.84257 },
    { 24, 1.84257 },
    { 25, 1.36839 },
    { 26, 1.68063 },
    { 27, 2.30398 },
    { 28, 1.83599 },
    { 29, 1.90419 },
    { 30, 2.06807 },
    { 31, 1.85088 },
    { 32, 2.53948 },
    { 33, 2.33758 },
    { 34, 2.49924 },
    { 35, 2.7111 },
    { 36, 2.95774 },
    { 37, 2.70158 },
    { 38, 2.22049 },
    { 39, 2.30927},
 { 40, 2.20199},
 { 41, 2.51324},
 { 42, 2.39785},
 { 43, 1.99402},
 { 44, 2.66471},
 { 45, 1.87621},
 { 46, 2.26679},
 { 49, 2.25402e-27},
 { 50, 0.436422},
 { 51, 0.347377},
 { 52, 0.930834},
 { 61, 0.57199},
 { 62, 0.711216},
 { 63, 0.78896},
 { 64, 0.780272},
 { 65, 0.857267},
 { 66, 0.861486},
 { 67, 1.33839},
 { 68, 1.0317},
 { 72, 0.557121},
 { 73, 0.557121},
 { 74, 0.653617},
 { 75, 0.830654},
 { 76, 0.665527}
}, 0);

    // planar
    curve_section b = curve_section({
//{ 1, 1.19417 },
//    { 2, 1.6759 },
//    { 3, 2.51467 },
//    { 4, 3.09769 },
//    { 5, 3.5692 },
//    { 9, 2.2175 },
//    { 10, 2.90137 },
//    { 11, 3.22125 },
//    { 12, 2.96254 },
//    { 13, 3.94092 },
//    { 17, 1.66036 },
//    { 18, 2.36509 },
//    { 19, 2.8563 },
//    { 20, 3.09211 },
//    { 21, 3.87213 },
//    { 22, 4.27582 },
//    { 23, 4.63555 },
//    { 24, 4.63555 },
//    { 25, 4.08617 },
//    { 26, 4.89313 },
//    { 27, 4.90371 },
//    { 28, 4.83851 },
//    { 29, 5.19495 },
//    { 30, 4.97734 },
//    { 31, 4.90597 },
//    { 32, 5.16705 },
//    { 33, 5.21779 },
//    { 34, 5.11933 },
//    { 35, 5.55157 },
    { 36, 6.30152e+36 },
    { 37, 5.12546 },
    { 38, 5.21992 },
    { 39, 4.8298 },
    { 40, 5.13875 },
    { 41, 5.35094 },
    { 42, 6.6722e+36 },
    { 43, 5.39129 },
    { 44, 5.21249 },
    { 45, 5.18866 },
    { 46, 5.42148 },
    { 49, 5.74787e-26 },
    { 50, 1.40446 },
    { 51, 1.98131 },
    { 52, 2.61235 },
    { 61, 1.50783 },
    { 62, 2.08656 },
    { 63, 2.26064 },
    { 64, 2.88338 },
    { 65, 3.14045 },
    { 66, 3.63071 },
    { 67, 3.67102 },
    { 68, 3.96318 },
    { 72, 1.70575 },
    { 73, 1.70575 },
    { 74, 2.32593 },
    { 75, 2.83022 },
    { 76, 3.02498 }
}, 0);

    return { a, b };

}


void increment_curve_values(std::list<curve_section>& graph, double increment) {

    for (auto& curve : graph)
        for (auto& point : curve)
            point.second += increment;
}

void save_test_svg(std::string_view const& filename)
{
    std::list<curve_section> graphs = get_sample_graphs();
    increment_curve_values(graphs, 50);

    std::list<curve_section> graphs_2 = get_sample_graphs_2();
    std::list<curve_section> graphs_2_a = { *graphs_2.begin() };
    auto s = graphs_2.begin();
    std::list<curve_section> graphs_2_b = { *(++s) };

    std::stringstream svg;

    svg << "<!DOCTYPE html>\n";
    svg << "<html>\n";
    svg << "<body>\n";

    //write_graph_as_svg(svg, Graph("Frame", "Cost", std::set<SplitCurve>({ {"Non-planar", graphs_2_a}, {"Planar", graphs_2_b} }), range_behaviour::no_max, 7, 70.0));
    write_graph_as_svg(svg, Graph("Frame", "Cost", std::set<SplitCurve>({  {"Planar", graphs_2_b} }), range_behaviour::no_max, 7, 70.0));

    //write_graph_as_svg(svg, Graph("Frame", "Map keyframe count", std::set<SplitCurve>({ {"Num keyframes", graphs} }), range_behaviour::split_by_stage, range_behaviour::no_max, 30.0));

    write_graph_as_svg(svg, Graph("Frame", "Map keyframe count", std::set<SplitCurve>({ {"Num keyframes", graphs} }), range_behaviour::no_max, range_behaviour::no_max, 70.0));

    write_graph_as_svg(svg, Graph("Frame", "Map keyframe count", std::set<SplitCurve>({ {"Num keyframes", graphs} }), range_behaviour::no_max, range_behaviour::no_min_0, 70.0));

    svg << "</body>\n";
    svg << "</html>\n";

    std::ofstream myfile;
    myfile.open(filename.data());
    myfile << svg.str();
    myfile.close();

}
