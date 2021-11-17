#include "MavDataNode.h"


MavDataNode::MavDataNode() : participant_(nullptr) , subscriber_(nullptr) , topic_(nullptr) , reader_(nullptr) , type_(new MavDataPubSubType())
{}



MavDataNode::~MavDataNode()
{
    if (reader_ != nullptr)
    {
        subscriber_->delete_datareader(reader_);
    }
    if (topic_ != nullptr)
    {
        participant_->delete_topic(topic_);
    }
    if (subscriber_ != nullptr)
    {
        participant_->delete_subscriber(subscriber_);
    }
    DomainParticipantFactory::get_instance()->delete_participant(participant_);
}


bool MavDataNode::init()
{
    DomainParticipantQos participantQos;
    participantQos.name("MavDataNode");
    participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

    if (participant_ == nullptr) { return false; }

    // Register the Type
    // Create the subscriptions Topic
    type_.register_type(participant_);
    topic_ = participant_->create_topic("MavDataTopic", "MavData", TOPIC_QOS_DEFAULT);
    if (topic_ == nullptr) { return false; }


    // Create the Subscriber
    subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
    if (subscriber_ == nullptr) { return false; }
    reader_ = subscriber_->create_datareader(topic_, DATAREADER_QOS_DEFAULT, &drListener_);
    if (reader_ == nullptr) { return false; }

    // Create the Publisher
    publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    if (publisher_ == nullptr) { return false; }
    writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, &dwListener_);
    if (writer_ == nullptr) { return false; }

    return true;
}


bool MavDataNode::publish(MavData* sendData)
{
    if (dwListener_.matched_ > 0)
    {
        writer_->write(sendData);
        return true;
    }
    return false;
}