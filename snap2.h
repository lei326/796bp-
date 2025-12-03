#ifndef SNAP_H
#define SNAP_H

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif
int SnapOneChannel(int ch);
int SnapChannels_0_to_5(void);
const char* SnapGetLastFilePath(int ch);
#ifdef __cplusplus
}
#endif

#endif // SNAP_H
