#pragma once

#include <boost/program_options.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/options_description.hpp>

struct cl_options {
  boost::program_options::variables_map vm;
  boost::program_options::options_description main_opts;
};

cl_options provide_cl_arguments(int argc, char **argv);
