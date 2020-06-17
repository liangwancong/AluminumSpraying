#pragma once
#include "paint_model.h"

namespace json2model {
	char * createLinesToJSON(std::vector<PaintPoints> paintPoints);
	int parsingJSONToKeyPoints(const char *monitor, MatchModel &matches);
	int parsingJSONToKeyPoints(const char *monitor, std::vector<MatchModelC> &v_matches);
	char * createKeyPointsToJSON(MatchModel &matches);
}