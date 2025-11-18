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
	mutable std::vector<Action> moveHistory;  // Track last moves to detect loops
	mutable int consecutiveRepeatedMoves = 0; // Count of repeated moves
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

class MoveToObjectOption : public Option {
public:
	MoveToObjectOption();
	const std::string& name() const override { return optionName; }
	void onSelect(Environment2D& env) override;
	bool isComplete(const Environment2D& env) const override;
	std::function<bool(const Environment2D&)> goal() const override;
	std::function<Action(const Environment2D&)> policy() const override;
	
	// Path storage for returning to target
	const std::vector<sf::Vector2i>& getPathToObject() const { return pathToObject; }
	void setPathToObject(const std::vector<sf::Vector2i>& path) { pathToObject = path; }
private:
	std::string optionName;
	std::vector<sf::Vector2i> pathToObject;
	mutable std::vector<Action> moveHistory;  // Track last moves
	mutable int consecutiveRepeatedMoves = 0;
};

class MoveObjectToTargetOption : public Option {
public:
	MoveObjectToTargetOption();
	const std::string& name() const override { return optionName; }
	void onSelect(Environment2D& env) override;
	bool isComplete(const Environment2D& env) const override;
	std::function<bool(const Environment2D&)> goal() const override;
	std::function<Action(const Environment2D&)> policy() const override;
	
	// Use the path from MoveToObjectOption to return
	void setReturnPath(const std::vector<sf::Vector2i>& path) { 
		returnPath = path; 
		returnPathIndex = 0;  // Reset index when path is set
	}
private:
	std::string optionName;
	sf::Vector2i objectPickupLocation;
	std::vector<sf::Vector2i> returnPath;
	mutable size_t returnPathIndex = 0;  // mutable to allow modification in const policy()
	mutable std::vector<Action> moveHistory;  // Track last moves
	mutable int consecutiveRepeatedMoves = 0;
};

class ReturnToObjectOption : public Option {
public:
	ReturnToObjectOption();
	const std::string& name() const override { return optionName; }
	void onSelect(Environment2D& env) override;
	bool isComplete(const Environment2D& env) const override;
	std::function<bool(const Environment2D&)> goal() const override;
	std::function<Action(const Environment2D&)> policy() const override;
private:
	std::string optionName;
	mutable std::vector<Action> moveHistory;  // Track last moves
	mutable int consecutiveRepeatedMoves = 0;
};

std::vector<std::unique_ptr<Option>> makeDefaultOptions();
