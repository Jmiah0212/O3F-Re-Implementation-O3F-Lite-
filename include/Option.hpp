#pragma once

#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <SFML/System/Vector2.hpp>

class Environment2D;

enum class Action;

class Option {
public:
	virtual ~Option() = default;
	virtual const std::string& name() const = 0;
	virtual void onSelect(Environment2D& env) = 0;
	virtual bool isComplete(const Environment2D& env) const = 0;
	virtual std::function<bool(const Environment2D&)> goal() const = 0;
	virtual std::function<Action(const Environment2D&)> policy() const = 0;
};

class MoveToTargetOption : public Option {
public:
	MoveToTargetOption();
	const std::string& name() const override { return optionName; }
	void onSelect(Environment2D& env) override;
	bool isComplete(const Environment2D& env) const override;
	std::function<bool(const Environment2D&)> goal() const override;
	std::function<Action(const Environment2D&)> policy() const override;
private:
	std::string optionName;
};

class ClearObstacleOption : public Option {
public:
	ClearObstacleOption();
	const std::string& name() const override { return optionName; }
	void onSelect(Environment2D& env) override;
	bool isComplete(const Environment2D& env) const override;
	std::function<bool(const Environment2D&)> goal() const override;
	std::function<Action(const Environment2D&)> policy() const override;
private:
	std::string optionName;
};

class GraspTargetOption : public Option {
public:
	GraspTargetOption();
	const std::string& name() const override { return optionName; }
	void onSelect(Environment2D& env) override;
	bool isComplete(const Environment2D& env) const override;
	std::function<bool(const Environment2D&)> goal() const override;
	std::function<Action(const Environment2D&)> policy() const override;
private:
	std::string optionName;
};

std::vector<std::unique_ptr<Option>> makeDefaultOptions();
