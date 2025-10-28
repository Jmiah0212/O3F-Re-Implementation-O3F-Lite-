#pragma once

#include <string>
#include <memory>
#include <vector>
#include <SFML/System/Vector2.hpp>

class Environment2D;

class Option {
public:
	virtual ~Option() = default;
	virtual const std::string& name() const = 0;
	virtual void onSelect(Environment2D& env) = 0;
	virtual bool isComplete(const Environment2D& env) const = 0;
};

class MoveToTargetOption : public Option {
public:
	MoveToTargetOption();
	const std::string& name() const override { return optionName; }
	void onSelect(Environment2D& env) override;
	bool isComplete(const Environment2D& env) const override;
private:
	std::string optionName;
};

class PushNearestObjectOption : public Option {
public:
	PushNearestObjectOption();
	const std::string& name() const override { return optionName; }
	void onSelect(Environment2D& env) override;
	bool isComplete(const Environment2D& env) const override;
private:
	std::string optionName;
	int activeObjectIndex;
};

std::vector<std::unique_ptr<Option>> makeDefaultOptions();
