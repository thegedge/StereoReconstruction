//---------------------------------------------------------------------
//
// Copyright © 2011, Jason Gedge <gedge -at- ualberta -dot- ca>
//
// This file is part of StereoReconstruction.
//
// StereoReconstruction is free software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// StereoReconstruction is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with StereoReconstruction. If not, see <http:www.gnu.org/licenses/>.
//
//---------------------------------------------------------------------
#ifndef TASK_HPP
#define TASK_HPP

#include <QEvent>
#include <QObject>
#include <QString>


FORWARD_DECLARE(Task);

//---------------------------------------------------------------------

/*!
 * An event that is sent whenever a new task is created
 */
class NewTaskEvent : public QEvent {
public:
	static const QEvent::Type TYPE = static_cast<QEvent::Type>(QEvent::User + 1);

public:
	NewTaskEvent(TaskPtr task)
		: QEvent(TYPE)
		, task_(task)
	{ }

	TaskPtr task() { return task_; }

private:
	TaskPtr task_;
};

//---------------------------------------------------------------------

/*!
 * An interface defining an object that emits progress updates.
 */
class Task : public QObject {
	Q_OBJECT

public:
	virtual ~Task() { }

public:
	//! A title given to what's happening (e.g., "Loading Items...")
	virtual QString title() const = 0;

	//! The number of steps involved in this task.
	virtual int numSteps() const = 0;

	//! Whether or not this task was cancelled
	bool isCancelled() const { return cancelled; }

public slots:
	/*!
	 * Cancel this task. Note that cancellation may not occur immediately,
	 * depending on the subclass' implementation.
	 */
	void cancel() {
		cancelled = true;
	}

	//! Run the task
	void run();

protected:
	//! To be implemented by subclasses (main entry point for code).
	virtual void runTask() = 0;

signals:
	//! Emitted when this task begins computation.
	void started(const Task *task);

	//! Emitted when this task finishes.
	void finished(const Task *task);

	//! Emitted when this task enters a new step of computation.
	void progressUpdate(int step);

	//! Emitted whenever the description of the current step changes.
	void stageUpdate(QString stage);

private:
	//! For use in subclasses, indicates whether or not this task is cancelled.
	bool cancelled;
};

//---------------------------------------------------------------------

#endif // TASK_HPP
