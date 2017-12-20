#include <rw/common/macros.hpp>
#include <rw/common/Log.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;

int main() {
    Log::infoLog() << "The using namespace enables us to call Log directly!\n";
    rw::common::Log::infoLog() << "We can still use the native namespace!\n";
    robwork::Log::infoLog() << "but also the general namespace!\n";
    return 0;
}
