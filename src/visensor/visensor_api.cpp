#include "visensor/visensor_api.hpp"
#include "visensor_impl.hpp"

namespace visensor
{

ViSensorDriver::ViSensorDriver()
{
  pImpl_ = new Impl();
}

void ViSensorDriver::getAutoDiscoveryDeviceList(ViDeviceList &hostname_list)
{
  pImpl_->getAutoDiscoveryDeviceList(hostname_list);
}

std::string ViSensorDriver::init()
{
  return pImpl_->initAutodiscovery();
}

void ViSensorDriver::init(std::string hostname)
{
  pImpl_->init(hostname);
}

ViSensorDriver::~ViSensorDriver()//:_fpga(&getSensorFromID){
{
  delete pImpl_;
}

void ViSensorDriver::setCameraCallback(boost::function<void (ViFrame::Ptr, ViErrorCode)> callback)
{
  pImpl_->setCameraCallback(callback);
}

void ViSensorDriver::setCornerCallback(boost::function<void(ViCorner::Ptr, ViErrorCode)> callback) {
  pImpl_->setCornerCallback(callback);
}

void ViSensorDriver::setFramesCornersCallback(boost::function<void(ViFrame::Ptr, ViCorner::Ptr)> callback) {
  pImpl_->setFramesCornersCallback(callback);
}

void ViSensorDriver::setImuCallback(boost::function<void (boost::shared_ptr<ViImuMsg>, ViErrorCode)> callback)
{
  pImpl_->setImuCallback(callback);
}

void ViSensorDriver::startAllCameras(uint32_t rate) {
  pImpl_->startAllCameras(rate);
}

void ViSensorDriver::startSensor(SensorId::SensorId id, uint32_t rate)
{
  pImpl_->startSensor(id, rate);
}

void ViSensorDriver::stopSensor(SensorId::SensorId id)
{
  pImpl_->stopSensor(id);
}

void ViSensorDriver::setSensorConfigParam(SensorId::SensorId sensor_id,
                                                  std::string cmd,
                                                  int value) {
  pImpl_->setSensorConfigParam(sensor_id, cmd, value);
}

void ViSensorDriver::startAllCorners() {
  pImpl_->startAllCorners();
}

void ViSensorDriver::startAllImus(uint32_t rate) {
  pImpl_->startAllImus(rate);
}

std::vector<SensorId::SensorId> ViSensorDriver::getListOfSensorIDs() const
{
  return pImpl_->getListOfSensorIDs();
}

std::vector<SensorId::SensorId> ViSensorDriver::getListOfCameraIDs() const
{
	return pImpl_->getListOfCameraIDs();
}

std::vector<SensorId::SensorId> ViSensorDriver::getListOfDenseIDs() const
{
  return pImpl_->getListOfDenseIDs();
}

std::vector<SensorId::SensorId> ViSensorDriver::getListOfCornerIDs() const {
  return pImpl_->getListOfCornerIDs();
}

std::vector<SensorId::SensorId> ViSensorDriver::getListOfImuIDs() const
{
	return pImpl_->getListOfImuIDs();
}

std::vector<SensorId::SensorId> ViSensorDriver::getListOfTriggerIDs() const
{
	return pImpl_->getListOfTriggerIDs();
}

uint32_t ViSensorDriver::getFpgaId() const
{
	return pImpl_->getFpgaId();
}

void ViSensorDriver::downloadFile(std::string local_path, std::string remote_path) {
  pImpl_->downloadFile(local_path, remote_path);
}

void ViSensorDriver::uploadFile(std::string local_path, std::string remote_path) {
  pImpl_->uploadFile(local_path, remote_path);
}

void ViSensorDriver::startAllExternalTriggers(uint32_t rate) {
  pImpl_->startAllExternalTriggers(rate);
}

void ViSensorDriver::setExternalTriggerCallback(
    boost::function<void(ViExternalTriggerMsg::Ptr)> callback) {
  pImpl_->setExternalTriggerCallback(callback);
}

void ViSensorDriver::setExternalTriggerConfig(const ViExternalTriggerConfig config) {
  pImpl_->setExternalTriggerConfig(config);
}

void ViSensorDriver::startAllDenseMatchers() {
  pImpl_->startAllDenseMatchers();
}

void ViSensorDriver::setDenseMatcherCallback(boost::function<void(ViFrame::Ptr, ViErrorCode)> callback) {
  pImpl_->setDenseMatcherCallback(callback);
}

void ViSensorDriver::setCameraCalibrationSlot(int slot_id){
  pImpl_->setCameraCalibrationSlot(slot_id);
}

int ViSensorDriver::getCameraCalibrationSlot(){
  return pImpl_->getCameraCalibrationSlot();
}

bool ViSensorDriver::getCameraCalibration(SensorId::SensorId cam_id, ViCameraCalibration &calib, int slot_id) const
{
  return pImpl_->getCameraCalibration(cam_id, slot_id, calib);
}

bool ViSensorDriver::setCameraCalibration(SensorId::SensorId cam_id, const ViCameraCalibration calib, int slot_id ) const
{
  return pImpl_->setCameraCalibration(cam_id, slot_id, calib);
}

bool ViSensorDriver::setCameraFactoryCalibration(SensorId::SensorId cam_id, const ViCameraCalibration calib) const
{
  return pImpl_->setCameraFactoryCalibration(cam_id, calib);
}

void ViSensorDriver::sendSerialData(ViSerialData::Ptr data) const
{
  pImpl_->sendSerialData(data);
}

void ViSensorDriver::setSerialCallback(boost::function<void(ViSerialData::Ptr)> callback)
{
  pImpl_->setSerialCallback(callback);
}

void ViSensorDriver::setSerialDelimiter(const char serial_id, const std::string delimiter) const
{
  pImpl_->setSerialDelimiter(serial_id, delimiter);
}

void ViSensorDriver::setSerialBaudrate(const char serial_id, const unsigned int baudrate) const
{
  pImpl_->setSerialBaudrate(serial_id, baudrate);
}

}  //namespace visensor
