#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#ifdef _WINDOWS
#include <io.h>
#endif

#include "io.hpp"

using namespace std;

namespace agv_robot
{
using google::protobuf::io::FileInputStream;
using google::protobuf::io::FileOutputStream;
using google::protobuf::io::ZeroCopyInputStream;
using google::protobuf::io::CodedInputStream;
using google::protobuf::io::ZeroCopyOutputStream;
using google::protobuf::io::CodedOutputStream;
using google::protobuf::Message;


bool ReadProtoFromTextFile(const char* filename, Message* proto)
{
	int fd = open(filename, O_RDONLY);
	CHECK_NE(fd, -1) << "File not found: " << filename;
	FileInputStream* input = new FileInputStream(fd);
	bool success = google::protobuf::TextFormat::Parse(input, proto);
	delete input;
	close(fd);
	return success;
}

void WriteProtoToTextFile(const Message& proto, const char* filename)
{
	int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	FileOutputStream* output = new FileOutputStream(fd);
	CHECK(google::protobuf::TextFormat::Print(proto, output));
	delete output;
	close(fd);
}
}
