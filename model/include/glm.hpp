#ifndef FLOCKEVO_GLM_HPP_INCLUDED
#define FLOCKEVO_GLM_HPP_INCLUDED

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/constants.hpp>

inline glm::vec2 limit(glm::vec2 const& x, float m)
{
  float l2 = glm::length2(x);
  return (l2 > 0.0f && l2 > m*m) ? (m / std::sqrt(l2)) * x : x;
}

#endif