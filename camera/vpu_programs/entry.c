#include <stdint.h>

void find_active_tiles(uint32_t, uint32_t, uint32_t, uint32_t);

uint32_t main(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3, uint32_t r4, uint32_t r5)
{
	switch (r0)
	{
		case 1:
		{
			uint32_t src	= (r1 & ~0xC0000000) | (3 << 30);
			uint32_t mask	= (r2 & ~0xC0000000) | (3 << 30);
			uint32_t dest	= (r3 & ~0xC0000000) | (3 << 30);

			find_active_tiles(src, mask, dest, r4);
			return 0;
		}
		default:
			return 1;
	}
}
