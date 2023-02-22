#include "plot_html.h"

#include <iomanip>
#include <sstream>
#include <array>
#include <fstream>
#include <vector>
#include <list>

struct LabelToValue {
    LabelToValue(std::string label, float value, int index)
        : label(label), value(value), index(index) {}
    LabelToValue()
        : value(0), index(-1) {}
    std::string label;
    float value;
    int index;
};

struct LabelledCurve {
    std::string name;
    std::vector<LabelToValue> dataValues;
};

void addLabelToValueGraphToHtml(std::stringstream& html, std::string const& title, std::string_view yLabel, std::string_view xLabel, std::list<LabelledCurve> const& curves, bool linearXLabels, int graphWidth = 1200, int graphHeight = 600, bool drawVertexCircles = true);

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

std::list<std::pair<float, std::string>> tickMarksForValueRange(float minVal, float maxVal, int tickCount)
{
   float range(maxVal - minVal);

   float tickSeparation(range / float(tickCount));
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
    for (unsigned int v = unsigned int(1 << 31); n != 0; n >>= 1, v ^= v >> 1)
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

void addLabelToValueGraphToHtml(std::stringstream& html, std::string const& title, std::string_view yLabel, std::string_view xLabel, std::list<LabelledCurve> const& curves, bool linearXLabels, int graphWidth, int graphHeight, bool drawVertexCircles) {
   int valueNameSize(200); // Size of text xLabel
   int labelSize(200); // size of text dataValues[i].label - todo calculate based on string length - maybe use 'textLength' attriubute to force size
   int legendWidth(400);

   int width(graphWidth), height(graphHeight);

   if (!title.empty())
      html << "<h3>" << title << "</h3>" << std::endl;

   html << "<svg height=\"" << height << "\" width=\"" << width + legendWidth << "\">" << std::endl;

   // Horizontal axis (x-value 0, 1, 2, 3...)
   int xCanvas0(30);
   int yCanvas0(height - labelSize); // bottom
   float xCanvasInc(0);
   float yCanvasInc(0);
   float xStartValue(0);
   if (!linearXLabels)
   {
      std::set<int> allTimeValues;
      std::map<int, std::string> timeValueToLabel;
      for (auto const& curve : curves)
         for (auto const& dataValue : curve.dataValues) {
            allTimeValues.insert(dataValue.index);
            timeValueToLabel[dataValue.index] = dataValue.label;
         }
      std::vector<std::string> labels(timeValueToLabel.size());
      int iLabel(0);
      for (auto const& label : timeValueToLabel) {
         labels[iLabel] = label.second;
         ++iLabel;
      }

      int xCanvasEnd(width - valueNameSize);
      html << "<text x=\"" << xCanvasEnd + 1 << "\" y=\"" << yCanvas0 << "\" fill=\"black\" >" << xLabel << "</text>" << std::endl;
      html << "<line x1=\"" << xCanvas0 << "\" y1=\"" << yCanvas0 << "\" x2=\"" << xCanvasEnd << "\" y2=\"" << yCanvas0 << "\" style=\"stroke:rgb(0,0,0);stroke-width:2\" />" << std::endl;
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

         html << "<line x1=\"" << tick.first << "\" y1=\"" << yCanvas0 << "\" x2=\"" << tick.first << "\" y2=\"" << yCanvas0 + 5 << "\" style=\"stroke:rgb(0,0,0);stroke-width:1\" />" << std::endl;
         float tx(tick.first + textOffset);
         float ty(yCanvas0 + 7);
         html << "<text x=\"" << tx << "\" y=\"" << ty << "\" fill=\"black\" transform=\"rotate(90 " << tx << ", " << ty << ")\">" << tick.second << "</text>" << std::endl;
      }
   }
   else // linearLabels
   {
      int approxTickCountX(20);

      // Get the min/max x value
      float minXValue(9999999), maxXValue(0);
      for (auto const& curve : curves) {
         for (auto const& dataValue : curve.dataValues) {
            if (maxXValue < dataValue.index)
               maxXValue = dataValue.index;
            if (minXValue > dataValue.index)
               minXValue = dataValue.index;
         }
      }
      std::list<std::pair<float, std::string>> ticks = tickMarksForValueRange(minXValue, maxXValue, approxTickCountX);

      //std::set<int> allTimeValues;
      //std::map<int, std::string> timeValueToLabel;
      //for (auto const& curve : curves)
      //   for (auto const& dataValue : curve.dataValues) {
      //      allTimeValues.insert(dataValue.index);
      //      timeValueToLabel[dataValue.index] = dataValue.label;
      //   }
      //std::vector<std::string> labels(timeValueToLabel.size());
      //int i(0);
      //for (auto const& label : timeValueToLabel) {
      //   labels[i] = label.second;
      //   ++i;
      //}

      int xCanvasEnd(width - valueNameSize);
      html << "<text x=\"" << xCanvasEnd + 1 << "\" y=\"" << yCanvas0 << "\" fill=\"black\" >" << xLabel << "</text>" << std::endl;
      html << "<line x1=\"" << xCanvas0 << "\" y1=\"" << yCanvas0 << "\" x2=\"" << xCanvasEnd << "\" y2=\"" << yCanvas0 << "\" style=\"stroke:rgb(0,0,0);stroke-width:2\" />" << std::endl;
      xCanvasInc = (float(xCanvasEnd - xCanvas0) / ((maxXValue - minXValue)*1.02f));
      xStartValue = minXValue - 0.01f*(maxXValue - minXValue);


      //int subsampling(1);
      //if (labels.size()>20) subsampling = 2;
      //if (labels.size()>50) subsampling = 5;
      //if (labels.size()>100) subsampling = 10;
      //if (labels.size()>200) subsampling = 20;
      //if (labels.size()>500) subsampling = 50;
      //std::list<std::pair<float, std::string>> ticks; // value markers on the horizontal axis
      //for (int i=0; i<labels.size(); ++i) {
      //   if (i==0 || ((i+1)%subsampling)==0) {
      //      int ep(i+1);
      //      float l(xCanvas0+float(ep)*xCanvasInc);
      //      ticks.push_back(std::pair<float, std::string>(l, labels[i]));
      //   }
      //}
      for (auto const& tick : ticks) {
         float textOffset(-4); // x offset
         //if (tick.second.size()>=1) textOffset = -8;
         //if (tick.second.size()>=2) textOffset = -12;
         //if (tick.second.size()>=3) textOffset = -16;

         float xPos(xCanvas0 + (tick.first - xStartValue)*xCanvasInc);

         html << "<line x1=\"" << xPos << "\" y1=\"" << yCanvas0 << "\" x2=\"" << xPos << "\" y2=\"" << yCanvas0 + 5 << "\" style=\"stroke:rgb(0,0,0);stroke-width:1\" />" << std::endl;
         float tx(xPos + textOffset);
         float ty(yCanvas0 + 7);
         html << "<text x=\"" << tx << "\" y=\"" << ty << "\" fill=\"black\" transform=\"rotate(90 " << tx << ", " << ty << ")\">" << tick.second << "</text>" << std::endl;
      }
   }

   // Vertical axis
   {
      int approxTickCountY(10);

      int yCanvasEnd(yLabel.empty() ? 0 : 25); // top
      html << "<text x=\"" << xCanvas0 - 25 << "\" y=\"" << yCanvasEnd - 10 << "\" fill=\"black\" >" << yLabel << "</text>" << std::endl;
      html << "<line x1=\"" << xCanvas0 << "\" y1=\"" << yCanvas0 << "\" x2=\"" << xCanvas0 << "\" y2=\"" << yCanvasEnd << "\" style=\"stroke:rgb(0,0,0);stroke-width:2\" />" << std::endl;

      // Get the max y value
      float maxYValue(0);
      for (auto const& curve : curves)
         for (auto const& dataValue : curve.dataValues)
            if (maxYValue < dataValue.value)
               maxYValue = dataValue.value;
      float minYValue(0);

      std::list<std::pair<float, std::string>> ticks = tickMarksForValueRange(minYValue, maxYValue, approxTickCountY);

      yCanvasInc = (float(yCanvasEnd - yCanvas0) / (float(maxYValue - minYValue)*1.02f));

      for (auto const& tick : ticks) {

         float textOffset(-10);
         if (tick.second.size() >= 2) textOffset = -18;
         if (tick.second.size() >= 3) textOffset = -26;
         if (tick.second.size() >= 4) textOffset = -30;

         float tickFirst(yCanvas0 + tick.first*yCanvasInc);

         html << "<line x1=\"" << xCanvas0 - 5 << "\" y1=\"" << tickFirst << "\" x2=\"" << xCanvas0 << "\" y2=\"" << tickFirst << "\" style=\"stroke:rgb(0,0,0);stroke-width:1\" />" << std::endl;
         html << "<text x=\"" << xCanvas0 + textOffset - 2 << "\" y=\"" << tickFirst + 5 << "\" fill=\"black\" >" << tick.second << "</text>" << std::endl;
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

      // Collect the curve's point positions in canvas space
      std::list<std::pair<float, float>> xyList;
      for (auto const& dataValue : curve.dataValues) {
         xyList.push_back({ xCanvas0 + (dataValue.index - xStartValue)*xCanvasInc,
                            yCanvas0 +  dataValue.value               *yCanvasInc });
      }

      // Plot the curve as a polyline
      html << "<polyline points=\"";
      for (auto const& xy : xyList)
         html << xy.first << "," << xy.second << " ";
      html << "\" style=\"fill:none;stroke:rgb(" << rgb[c][0] << ", " << rgb[c][1] << ", " << rgb[c][2] << ");stroke-width:3\" />" << std::endl;

      // Draw a circle on each vertex of the curve
      if (drawVertexCircles)
         for (auto const& xy : xyList)
            html << "<circle cx = \"" << xy.first << "\" cy = \"" << xy.second << "\" r = \"3\" fill = rgb(" << rgb[c][0] << ", " << rgb[c][1] << ", " << rgb[c][2] << ") />" << std::endl;

      ++c;
   }

   // Add a legend
   int y(30);
   c = 0;
   for (auto const& curve : curves) {
      html << "<text x=\"" << width << "\" y=\"" << y << "\" style=\"fill:rgb(" << rgb[c][0] << ", " << rgb[c][1] << ", " << rgb[c][2] << ");\">" << curve.name << "</text>" << std::endl;
      ++c;
      y += 30;
   }

   html << "Sorry, your browser does not support inline SVG." << std::endl;
   html << "</svg>" << std::endl;
}

using Curve = std::pair<std::string, std::map<double, double>>;
using Graph = std::tuple<std::string, std::string, std::set<Curve>>;

void write_graphs_html(std::string_view const& filename, std::set<Graph> graphs)
{
   std::ofstream myfile;
   myfile.open(filename.data());

   std::stringstream html;
   html << "<!DOCTYPE html><html><head></head><body>";

   for (auto const& graph : graphs) {
    
      auto [xLabel, yLabel, curves] = graph; 
      std::list<LabelledCurve> labelled_curves;

      for (auto const& curve : curves) {
         LabelledCurve labelled_curve;
         labelled_curve.name = curve.first;
         labelled_curve.dataValues.resize(curve.second.size());
         int i(0);
         for (auto const& vertex : curve.second) {
            labelled_curve.dataValues[i].index = int(vertex.first + 0.5);
            labelled_curve.dataValues[i].value = (float)vertex.second;
            //labelled_curve.dataValues[i].label = std::string();
            ++i;
         }
         labelled_curves.push_back(labelled_curve);
      }
      bool linearXLabels(true);
      addLabelToValueGraphToHtml(html, yLabel, "", xLabel, labelled_curves, linearXLabels, 1200, 600, true);
   }

   html << "</body></html>";

   myfile << html.str();
   myfile.close();
}

bool disable_all_html_graph_export() {
   return true;
}

namespace stella_vslam_bfx {

metrics_and_debugging* metrics_and_debugging::get_instance()
{
    if (!instance)
        instance = new metrics_and_debugging();
    return instance;
}

void metrics_and_debugging::set_thread_name(std::string name)
{
    std::thread::id id = std::this_thread::get_id();
    auto f = thread_id_to_name.find(id);
    if (f == thread_id_to_name.end())
       thread_id_to_name[id] = name;
}

std::string metrics_and_debugging::thread_name() const
{
    auto f = thread_id_to_name.find(std::this_thread::get_id());
    if (f != thread_id_to_name.end())
        return f->second;
    return "Unregistered";
}

} // namespace stella_vslam_bfx