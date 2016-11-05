
#include <find_button/FindButton.h>


FindButton::FindButton(src_perception::MultisenseImage *handle)
{
	assert(handle != nullptr && "Multisesnse pointer is null");
	mi = handle;
}

