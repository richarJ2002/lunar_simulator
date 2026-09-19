/*!
 * @File:         spin.cc
 *
 * @Brief:        Dispatches every owned node's callbacks until shutdown is
 *                requested.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AlphaNode.h"

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_node
{

void AlphaNode::spin()
{
    executor.spin();
}

} /* namespace systems::alpha::alpha_node */
