#include "Keyframe.h"

Keyframe::Keyframe() {
  
}

Keyframe::Keyframe(long position, float speed) {
  this->position = position;
  this->speed = speed;
}

void Keyframe::setPosition(long position) {
  this->position = position;
}

long Keyframe::getPosition() {
  return this->position;
}

void Keyframe::setSpeed(float speed) {
  this->speed = speed;
}

float Keyframe::getSpeed() {
  return this->speed;
}

void Keyframe::setAcc(float acc) {
  this->acc = acc;
}

float Keyframe::getAcc() {
  return this->acc;
}

void Keyframe::setDec(float dec) {
  this->dec = dec;
}

float Keyframe::getDec() {
  return this->dec;
}

void Keyframe::setNextKeyframe(Keyframe* kf) {
  nextKeyframe = kf;
}

Keyframe* Keyframe::getNextKeyframe() {
  return nextKeyframe;
}

void Keyframe::setPreviousKeyframe(Keyframe* kf) {
  previousKeyframe = kf;  
}

Keyframe* Keyframe::getPreviousKeyframe() {
  return previousKeyframe;
}

