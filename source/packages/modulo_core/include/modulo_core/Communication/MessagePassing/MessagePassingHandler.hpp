/**
 * @author Baptiste Busch
 * @date 2020/02/13
 */
#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "modulo_core/Communication/CommunicationHandler.hpp"
#include "modulo_core/Communication/MessagePassing/ReadStateConversion.hpp"
#include "modulo_core/Communication/MessagePassing/WriteStateConversion.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			namespace MessagePassing
			{
				/**
				 * @class CommunicationHandler
				 * @brief Abstract class to define a MessagePassingHandler type communication interface
				 */
				class MessagePassingHandler : public CommunicationHandler
				{
				private:
					std::shared_ptr<StateRepresentation::State> recipient_; ///< recipient associated to the handler

				public:
					/**
					 * @brief Constructor for a CommunicationHandler
					 * @param  type      the type of CommunicationHandler from the CommunicationType enumeration
					 * @param  recipient recipient associated to the handler
					 * @param  timeout   period before considered time out
					 * @param  mutex     reference to the Cell mutex
					 */
					template <typename DurationT>
					explicit MessagePassingHandler(const CommunicationType& type,  const std::shared_ptr<StateRepresentation::State>& recipient, const std::chrono::duration<int64_t, DurationT>& timeout, const std::shared_ptr<std::mutex>& mutex);

					/**
					 * @brief Getter of the recipient
					 * @return the recipient
					 */
					const StateRepresentation::State& get_recipient() const;
				
					/**
					 * @brief Getter of the recipient as a non const value
					 * @return the recipient
					 */
					StateRepresentation::State& get_recipient();
				};

				template <typename DurationT>
				MessagePassingHandler::MessagePassingHandler(const CommunicationType& type, const std::shared_ptr<StateRepresentation::State>& recipient, const std::chrono::duration<int64_t, DurationT>& timeout, const std::shared_ptr<std::mutex>& mutex):
				CommunicationHandler(type, timeout, mutex),
				recipient_(recipient)
				{}

				inline const StateRepresentation::State& MessagePassingHandler::get_recipient() const
				{
					return *this->recipient_;
				}

				inline StateRepresentation::State& MessagePassingHandler::get_recipient()
				{
					return *this->recipient_;
				}
			}
		}
	}
}
