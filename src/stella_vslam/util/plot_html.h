#pragma once

#include <string>
#include <vector>
#include <list>

struct LabelToValue
{
   LabelToValue(std::string label, float value, int index) : label(label), value(value), index(index) {}
   LabelToValue() : value(0), index(-1) {}
   std::string label;
   float value;
   int index;
};

struct LabelledCurve
{
   std::string               name;
   std::vector<LabelToValue> dataValues;
};

void addLabelToValueGraphToHtml(std::stringstream& html, std::string const& title, std::string_view yLabel, std::string_view xLabel, std::list<LabelledCurve> const& curves, bool linearXLabels, int graphWidth = 1200, int graphHeight = 600, bool drawVertexCircles = true);


