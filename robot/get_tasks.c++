#include <curl/curl.h>
#include <string>
#include <iostream>

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
		((std::string*)userp)->append((char*)contents, size * nmemb);
		return size * nmemb;
}

int main(void) {

		CURL *curl;
		CURLcode res;
		std::string readBuffer;
		curl = curl_easy_init();

		if (curl) {
				curl_easy_setopt(curl, CURLOPT_URL, "http://localhost/script/get_tasks.php");
				curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
				curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
				res = curl_easy_perform(curl);
				curl_easy_cleanup(curl);
				std::cout << readBuffer << std::endl;
		}
		return 0;
}
