#include "pcl/outofcore/cJSON.h"
#include "json_model_parse.h"

namespace json2model {
	using namespace std;

	char * createLinesToJSON(vector<PaintPoints> paintPoints)
	{
		char *string = NULL;

		cJSON *json_group_points = NULL;
		cJSON *json_paint_points = NULL;
		cJSON *json_pic_name = NULL;
		cJSON *json_match_name = NULL;
		cJSON *json_match_angle = NULL;
		cJSON *json_match_val = NULL;
		cJSON *json_match_mat = NULL;
		cJSON *json_stable_point = NULL;
		cJSON *json_have_plane = NULL;
		cJSON *json_points = NULL;
		cJSON *json_point = NULL;
		cJSON *json_point_x = NULL;
		cJSON *json_point_y = NULL;
		cJSON *json_point_angle = NULL;
		cJSON *json_point_need_change = NULL;
		cJSON *json_maxLine_start = NULL;
		cJSON *json_maxLine_start_x = NULL;
		cJSON *json_maxLine_start_y = NULL;
		cJSON *json_maxLine_end = NULL;
		cJSON *json_maxLine_end_x = NULL;
		cJSON *json_maxLine_end_y = NULL;
		cJSON *json_maxLine_content_points = NULL;
		cJSON *json_maxLine_content_point = NULL;
		cJSON *json_maxLine_content_point_x = NULL;
		cJSON *json_maxLine_content_point_y = NULL;
		cJSON *json_maxLine_content_angle = NULL;
		cJSON *json_maxLine_content_need_change = NULL;
		cJSON *monitor = cJSON_CreateObject();
		if (monitor == NULL)
		{
			goto end;
		}

		json_group_points = cJSON_CreateArray();
		if (json_group_points == NULL)
		{
			goto end;
		}
		cJSON_AddItemToObject(monitor, "paint_points", json_group_points);


		for (size_t i = 0; i < paintPoints.size(); i++)
		{
			PaintPoints tmp_paint_points = paintPoints[i];
			vector<PaintPoint> tmp_points = tmp_paint_points.points;

			json_paint_points = cJSON_CreateObject();
			if (json_paint_points == NULL)
			{
				goto end;
			}
			cJSON_AddItemToArray(json_group_points, json_paint_points);

			json_points = cJSON_CreateArray();
			if (json_points == NULL)
			{
				goto end;
			}

			cJSON_AddItemToObject(json_paint_points, "points", json_points);
			for (size_t j = 0; j < tmp_points.size(); j++)
			{
				PaintPoint tmp_point = tmp_points[j];

				json_point = cJSON_CreateObject();
				if (json_point == NULL)
				{
					goto end;
				}
				cJSON_AddItemToArray(json_points, json_point);

				json_point_x = cJSON_CreateNumber(tmp_point.point.x);
				if (json_point_x == NULL)
				{
					goto end;
				}
				cJSON_AddItemToObject(json_point, "x", json_point_x);

				json_point_y = cJSON_CreateNumber(tmp_point.point.y);
				if (json_point_y == NULL)
				{
					goto end;
				}
				cJSON_AddItemToObject(json_point, "y", json_point_y);

				json_point_angle = cJSON_CreateNumber(-tmp_point.angle);//因为c#外部要用的角度是负的所以这里取负
				if (json_point_angle == NULL)
				{
					goto end;
				}
				cJSON_AddItemToObject(json_point, "angle", json_point_angle);

				json_point_need_change = cJSON_CreateBool(tmp_point.needChange);
				if (json_point_need_change == NULL)
				{
					goto end;
				}
				cJSON_AddItemToObject(json_point, "need_change", json_point_need_change);
			}

			json_pic_name = cJSON_CreateString(tmp_paint_points.picName.data());
			if (json_pic_name == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_paint_points, "pic_name", json_pic_name);

			json_match_name = cJSON_CreateString(tmp_paint_points.matchName.data());
			if (json_match_name == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_paint_points, "match_name", json_match_name);

			json_match_mat = cJSON_CreateString(tmp_paint_points.matchMat.data());
			if (json_match_mat == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_paint_points, "match_mat", json_match_mat);

			json_match_angle = cJSON_CreateNumber(tmp_paint_points.matchAngle);
			if (json_match_angle == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_paint_points, "match_angle", json_match_angle);

			json_match_val = cJSON_CreateNumber(tmp_paint_points.matchVal);
			if (json_match_val == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_paint_points, "match_val", json_match_val);

			json_stable_point = cJSON_CreateObject();
			if (json_point == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_paint_points, "stable_point", json_stable_point);

			json_point_x = cJSON_CreateNumber(tmp_paint_points.stablePoint.x);
			if (json_point_x == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_stable_point, "x", json_point_x);

			json_point_y = cJSON_CreateNumber(tmp_paint_points.stablePoint.y);
			if (json_point_y == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_stable_point, "y", json_point_y);

			json_have_plane = cJSON_CreateBool(tmp_paint_points.havePlane);
			if (json_have_plane == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_paint_points, "have_plane", json_have_plane);

			{

				json_maxLine_start = cJSON_CreateObject();
				if (json_maxLine_start == NULL)
				{
					goto end;
				}
				cJSON_AddItemToObject(json_paint_points, "maxLine_start", json_maxLine_start);
				//int midIndex = floor(paintPoints.size() * 1.0 / 2.0);
				json_maxLine_start_x = cJSON_CreateNumber(paintPoints[i].maxLineStart.x);
				if (json_maxLine_start_x == NULL)
				{
					goto end;
				}
				cJSON_AddItemToObject(json_maxLine_start, "x", json_maxLine_start_x);
				json_maxLine_start_y = cJSON_CreateNumber(paintPoints[i].maxLineStart.y);
				if (json_maxLine_start_y == NULL)
				{
					goto end;
				}
				cJSON_AddItemToObject(json_maxLine_start, "y", json_maxLine_start_y);


				json_maxLine_end = cJSON_CreateObject();
				if (json_maxLine_end == NULL)
				{
					goto end;
				}
				cJSON_AddItemToObject(json_paint_points, "maxLine_end", json_maxLine_end);
				json_maxLine_end_x = cJSON_CreateNumber(paintPoints[i].maxLineEnd.x);
				if (json_maxLine_end_x == NULL)
				{
					goto end;
				}
				cJSON_AddItemToObject(json_maxLine_end, "x", json_maxLine_end_x);
				json_maxLine_end_y = cJSON_CreateNumber(paintPoints[i].maxLineEnd.y);
				if (json_maxLine_end_y == NULL)
				{
					goto end;
				}
				cJSON_AddItemToObject(json_maxLine_end, "y", json_maxLine_end_y);


				json_maxLine_content_points = cJSON_CreateArray();
				if (json_maxLine_content_points == NULL)
				{
					goto end;
				}

				cJSON_AddItemToObject(json_paint_points, "maxLine_content_points", json_maxLine_content_points);
				vector<PaintPoint> maxLineContentPoints = paintPoints[i].maxLineContentPoints;

				for (size_t j = 0; j < maxLineContentPoints.size(); j++)
				{
					PaintPoint tmp_point = maxLineContentPoints[j];
					//PaintPoint tmp_point(cv::Point2f(0,0),2,false);
					json_maxLine_content_point = cJSON_CreateObject();
					if (json_maxLine_content_point == NULL)
					{
						goto end;
					}
					cJSON_AddItemToArray(json_maxLine_content_points, json_maxLine_content_point);

					json_maxLine_content_point_x = cJSON_CreateNumber(tmp_point.point.x);
					if (json_maxLine_content_point_x == NULL)
					{
						goto end;
					}
					cJSON_AddItemToObject(json_maxLine_content_point, "x", json_maxLine_content_point_x);

					json_maxLine_content_point_y = cJSON_CreateNumber(tmp_point.point.y);
					if (json_maxLine_content_point_y == NULL)
					{
						goto end;
					}
					cJSON_AddItemToObject(json_maxLine_content_point, "y", json_maxLine_content_point_y);

					json_maxLine_content_angle = cJSON_CreateNumber(-tmp_point.angle);//因为c#外部要用的角度是负的所以这里取负
					if (json_maxLine_content_angle == NULL)
					{
						goto end;
					}
					cJSON_AddItemToObject(json_maxLine_content_point, "angle", json_maxLine_content_angle);

					json_maxLine_content_need_change = cJSON_CreateBool(tmp_point.needChange);
					if (json_maxLine_content_need_change == NULL)
					{
						goto end;
					}
					cJSON_AddItemToObject(json_maxLine_content_point, "change", json_maxLine_content_need_change);
				}
			}
		}

		string = cJSON_Print(monitor);
		if (string == NULL)
		{
			fprintf(stderr, "Failed to print monitor.\n");
		}

	end:
		cJSON_Delete(monitor);
		return string;
	}

	int parsingJSONToKeyPoints(const char *monitor, MatchModel &matches) {

		cJSON *json_points = NULL;
		cJSON *json_point = NULL;
		cJSON *json_point_x = NULL;
		cJSON *json_point_y = NULL;
		cJSON *json_point_angle = NULL;
		cJSON *json_point_need_change = NULL;
		cJSON *json_max_height = NULL;
		cJSON *json_stable_point = NULL;

		int status = 1;

		vector<PaintPoint> paintPoints;

		cJSON *monitor_json = cJSON_Parse(monitor);
		if (monitor_json == NULL)
		{
			const char *error_ptr = cJSON_GetErrorPtr();
			if (error_ptr != NULL)
			{
				fprintf(stderr, "Error before: %s\n", error_ptr);
			}
			status = 0;
			goto end;
		}
		int a = cJSON_Number;

		json_max_height = cJSON_GetObjectItem(monitor_json, "max_height");

		if (json_max_height->type != cJSON_Number) {
			status = 0;
			goto end;
		}
		matches.max_height = json_max_height->valuedouble;


		json_stable_point = cJSON_GetObjectItem(monitor_json, "stable_point");
		if (json_stable_point != NULL) {
			cJSON *json_point_x = cJSON_GetObjectItem(json_stable_point, "x");
			cJSON *json_point_y = cJSON_GetObjectItem(json_stable_point, "y");
			if (json_point_x->type == cJSON_Number && json_point_y->type == cJSON_Number) {
				matches.stable_point = cv::Point(json_point_x->valueint, json_point_y->valueint);
			}
		}

		json_points = cJSON_GetObjectItem(monitor_json, "points");
		for (size_t i = 0; i < cJSON_GetArraySize(json_points); i++)
		{
			cJSON *json_point = cJSON_GetArrayItem(json_points, i);
			cJSON *json_point_x = cJSON_GetObjectItem(json_point, "x");
			cJSON *json_point_y = cJSON_GetObjectItem(json_point, "y");
			cJSON *json_point_angle = cJSON_GetObjectItem(json_point, "angle");
			cJSON *json_point_need_change = cJSON_GetObjectItem(json_point, "need_change");



			if (json_point_x->type != cJSON_Number || json_point_y->type != cJSON_Number || json_point_angle->type != cJSON_Number || (json_point_need_change->type != cJSON_False && json_point_need_change->type != cJSON_True))
			{
				status = 0;
				goto end;
			}

			PaintPoint paintPoint = PaintPoint(cv::Point2f(json_point_x->valuedouble, json_point_y->valuedouble), json_point_angle->valuedouble, json_point_need_change->valueint);
			paintPoints.push_back(paintPoint);

		}
		matches.points = paintPoints;
	end:
		cJSON_Delete(monitor_json);
		return status;
	}
	int parsingJSONToKeyPoints(cJSON *monitor_json, MatchModelC &matches) {

		cJSON *json_points = NULL;
		cJSON *json_point = NULL;
		cJSON *json_point_x = NULL;
		cJSON *json_point_y = NULL;
		cJSON *json_point_angle = NULL;
		cJSON *json_point_need_change = NULL;
		cJSON *json_have_plane = NULL;
		cJSON *json_stable_point = NULL;
		cJSON *json_match_name = NULL;
		cJSON *json_match_mat = NULL;
		cJSON *json_pic_name = NULL;
		cJSON *json_match_angle = NULL;
		
		int status = 1;

		vector<PaintPoint> paintPoints;

		//cJSON *monitor_json = cJSON_Parse(monitor);
		//if (monitor_json == NULL)
		//{
		//	const char *error_ptr = cJSON_GetErrorPtr();
		//	if (error_ptr != NULL)
		//	{
		//		fprintf(stderr, "Error before: %s\n", error_ptr);
		//	}
		//	status = 0;
		//	goto end;
		//}
		int a = cJSON_Number;

		json_have_plane = cJSON_GetObjectItem(monitor_json, "have_plane");

		if ((json_have_plane->type != cJSON_False && json_have_plane->type != cJSON_True)) {
			status = 0;
			goto end;
		}

		matches.have_plane = json_have_plane->valueint;

		json_match_name = cJSON_GetObjectItem(monitor_json, "match_name");

		if (json_match_name->type != cJSON_String) {
			status = 0;
			goto end;
		}

		matches.match_name = json_match_name->valuestring;

		json_match_mat = cJSON_GetObjectItem(monitor_json, "match_mat");

		if (json_match_mat->type != cJSON_String) {
			status = 0;
			goto end;
		}

		matches.match_mat = json_match_mat->valuestring;

		json_pic_name = cJSON_GetObjectItem(monitor_json, "pic_name");

		if (json_pic_name->type != cJSON_String) {
			status = 0;
			goto end;
		}

		matches.pic_name = json_pic_name->valuestring;

		json_match_angle = cJSON_GetObjectItem(monitor_json, "match_angle");

		if (json_match_angle->type != cJSON_Number) {
			status = 0;
			goto end;
		}

		matches.match_angle = json_match_angle->valuedouble;


		json_stable_point = cJSON_GetObjectItem(monitor_json, "stable_point");
		if (json_stable_point != NULL) {
			cJSON *json_point_x = cJSON_GetObjectItem(json_stable_point, "x");
			cJSON *json_point_y = cJSON_GetObjectItem(json_stable_point, "y");
			if (json_point_x->type == cJSON_Number && json_point_y->type == cJSON_Number) {
				matches.stable_point = cv::Point(json_point_x->valueint, json_point_y->valueint);
			}
		}

		json_points = cJSON_GetObjectItem(monitor_json, "points");
		for (size_t i = 0; i < cJSON_GetArraySize(json_points); i++)
		{
			cJSON *json_point = cJSON_GetArrayItem(json_points, i);
			cJSON *json_point_x = cJSON_GetObjectItem(json_point, "x");
			cJSON *json_point_y = cJSON_GetObjectItem(json_point, "y");
			cJSON *json_point_angle = cJSON_GetObjectItem(json_point, "angle");
			cJSON *json_point_need_change = cJSON_GetObjectItem(json_point, "need_change");



			if (json_point_x->type != cJSON_Number || json_point_y->type != cJSON_Number || json_point_angle->type != cJSON_Number || (json_point_need_change->type != cJSON_False && json_point_need_change->type != cJSON_True))
			{
				status = 0;
				goto end;
			}

			PaintPoint paintPoint = PaintPoint(cv::Point2f(json_point_x->valuedouble, json_point_y->valuedouble), json_point_angle->valuedouble, json_point_need_change->valueint);
			paintPoints.push_back(paintPoint);

		}
		matches.points = paintPoints;
	end:
		//cJSON_Delete(monitor_json);
		return status;
	}
	int parsingJSONToKeyPoints(const char *monitor, std::vector<MatchModelC> &v_matches) {

	cJSON *json_paint_points = NULL;
		int status = 1;

		vector<PaintPoint> paintPoints;

		cJSON *monitor_json = cJSON_Parse(monitor);
		if (monitor_json == NULL)
		{
			const char *error_ptr = cJSON_GetErrorPtr();
			if (error_ptr != NULL)
			{
				fprintf(stderr, "Error before: %s\n", error_ptr);
			}
			status = 0;
			goto end;
		}
		json_paint_points = cJSON_GetObjectItem(monitor_json, "paint_points");
		for (size_t i = 0; i < cJSON_GetArraySize(json_paint_points); i++) {
			MatchModelC matches;
			cJSON *json_paint_point = cJSON_GetArrayItem(json_paint_points, i);
			status = parsingJSONToKeyPoints(json_paint_point, matches);
			if (status == 0) {
				goto end;
			}
			v_matches.push_back(matches);
		}

	end:
		cJSON_Delete(monitor_json);
		return status;
	}

	char * createKeyPointsToJSON(MatchModel &matches)
	{
		char *string = NULL;

		cJSON *json_max_height = NULL;
		cJSON *json_points = NULL;
		cJSON *json_point = NULL;
		cJSON *json_point_x = NULL;
		cJSON *json_point_y = NULL;
		cJSON *json_point_angle = NULL;
		cJSON *json_point_need_change = NULL;
		cJSON *json_stable_point = NULL;

		cJSON *monitor = cJSON_CreateObject();
		if (monitor == NULL)
		{
			goto end;
		}

		json_max_height = cJSON_CreateNumber(matches.max_height);
		if (json_max_height == NULL)
		{
			goto end;
		}
		cJSON_AddItemToObject(monitor, "max_height", json_max_height);

		json_stable_point = cJSON_CreateObject();
		if (json_stable_point != NULL)
		{
			cJSON_AddItemToObject(monitor, "stable_point", json_stable_point);

			json_point_x = cJSON_CreateNumber(matches.stable_point.x);
			if (json_point_x == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_stable_point, "x", json_point_x);

			json_point_y = cJSON_CreateNumber(matches.stable_point.y);
			if (json_point_y == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_stable_point, "y", json_point_y);
		}


		json_points = cJSON_CreateArray();
		if (json_points == NULL)
		{
			goto end;
		}
		cJSON_AddItemToObject(monitor, "points", json_points);
		for (size_t i = 0; i < matches.points.size(); i++)
		{
			PaintPoint tmp_point = matches.points[i];

			json_point = cJSON_CreateObject();
			if (json_point == NULL)
			{
				goto end;
			}
			cJSON_AddItemToArray(json_points, json_point);

			json_point_x = cJSON_CreateNumber(tmp_point.point.x);
			if (json_point_x == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_point, "x", json_point_x);

			json_point_y = cJSON_CreateNumber(tmp_point.point.y);
			if (json_point_y == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_point, "y", json_point_y);

			json_point_angle = cJSON_CreateNumber(tmp_point.angle);
			if (json_point_angle == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_point, "angle", json_point_angle);

			json_point_need_change = cJSON_CreateBool(tmp_point.needChange);
			if (json_point_need_change == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_point, "need_change", json_point_need_change);
		}


		string = cJSON_Print(monitor);
		if (string == NULL)
		{
			fprintf(stderr, "Failed to print monitor.\n");
		}

	end:
		cJSON_Delete(monitor);
		return string;
	}


	char * createMsgsToJSON(vector<MatchMsg> match_msgs)
	{
		char *string = NULL;
		cJSON *json_group = NULL;
		cJSON *json_match_msgs = NULL;
		cJSON *json_pic_name = NULL;
		cJSON *json_scale= NULL;
		cJSON *json_match_mat = NULL;
		cJSON *json_match_angle = NULL;
		cJSON *monitor = cJSON_CreateObject();
		if (monitor == NULL)
		{
			goto end;
		}

		json_group = cJSON_CreateArray();
		if (json_group == NULL)
		{
			goto end;
		}
		cJSON_AddItemToObject(monitor, "match_msgs", json_group);


		for (size_t i = 0; i < match_msgs.size(); i++)
		{

			json_match_msgs = cJSON_CreateObject();
			if (json_match_msgs == NULL)
			{
				goto end;
			}
			cJSON_AddItemToArray(json_group, json_match_msgs);

			json_pic_name = cJSON_CreateString(match_msgs[i].picName.data());
			if (json_pic_name == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_match_msgs, "pic_name", json_pic_name);

			json_scale = cJSON_CreateNumber(match_msgs[i].scale);
			if (json_scale == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_match_msgs, "scale", json_scale);

			json_match_mat = cJSON_CreateString(match_msgs[i].matchMat.data());
			if (json_match_mat == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_match_msgs, "match_mat", json_match_mat);

			json_match_angle = cJSON_CreateNumber(match_msgs[i].matchAngle);
			if (json_match_angle == NULL)
			{
				goto end;
			}
			cJSON_AddItemToObject(json_match_msgs, "match_angle", json_match_angle);
		}

		string = cJSON_Print(monitor);
		if (string == NULL)
		{
			fprintf(stderr, "Failed to print monitor.\n");
		}

	end:
		cJSON_Delete(monitor);
		return string;
	}

}